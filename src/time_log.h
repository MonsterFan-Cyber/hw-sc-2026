#pragma once

// TimeLog 单头文件实现
// - 提供宏接口：TL_SET_OUTPUT, TL_START, TL_END, TL_SCOPE
// - 非线程安全（接口处有中文注释）
// - 定义 TIME_LOG_DISABLE 时，所有接口变为空实现

#include <string>

#ifdef TIME_LOG_DISABLE

// 禁用时宏化为无操作，尽量不引入额外头文件
#define TL_SET_OUTPUT(path) (void)0
#define TL_START(label) (void)0
#define TL_END() (void)0
#define TL_SCOPE(label) (void)0
#define TL_ATTACH(info) (void)0

#else // TIME_LOG_DISABLE not defined

#include <chrono>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <ctime>
#include <filesystem>

namespace tl {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct Node {
	struct Event {
		enum class Kind {
			Attachment,
			Child
		};
		Kind kind;
		TimePoint time;
		std::string text;
		Node* child = nullptr;
	};

	std::string label;
	TimePoint start;
	TimePoint end;
	double duration_ms = 0.0;
	bool closed = false;
	std::vector<Event> events;
	Node* parent = nullptr;
};

// 输出文件路径（可以通过 TL_SET_OUTPUT 修改）
inline std::string time_log_output_path = "time_log.txt";

// 当前打开的时间层栈（非线程安全）
inline std::vector<Node*> tl_stack;

// 所有节点的所有权容器，避免指针失效
inline std::vector<std::unique_ptr<Node>> tl_nodes;

// 根节点按开始顺序保存，便于按层级输出
inline std::vector<Node*> tl_roots;

inline void tl_set_output(const std::string &path) {
	time_log_output_path = path;
}

inline Node* tl_make_node(const std::string &label) {
	auto node = std::make_unique<Node>();
	node->label = label;
	node->start = Clock::now();
	Node* raw = node.get();
	tl_nodes.push_back(std::move(node));
	if (tl_stack.empty()) {
		tl_roots.push_back(raw);
	} else {
		raw->parent = tl_stack.back();
		tl_stack.back()->events.push_back(Node::Event{Node::Event::Kind::Child, raw->start, {}, raw});
	}
	tl_stack.push_back(raw);
	return raw;
}

inline Node* tl_make_node(const char *label) {
	return tl_make_node(std::string(label));
}

inline void tl_start(const std::string &label) {
	tl_make_node(label);
}

inline void tl_start(const char *label) {
	tl_make_node(label);
}

inline void tl_attach(const std::string &info) {
	if (tl_stack.empty()) return;
	auto now = Clock::now();
	tl_stack.back()->events.push_back(Node::Event{Node::Event::Kind::Attachment, now, info, nullptr});
}

inline void tl_attach(const char *info) {
	tl_attach(std::string(info));
}

inline void tl_end() {
	if (tl_stack.empty()) return;
	TimePoint t2 = Clock::now();
	Node* p = tl_stack.back();
	tl_stack.pop_back();
	p->end = t2;
	p->duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - p->start).count();
	p->closed = true;
}

inline void tl_force_close_open_nodes() {
	TimePoint now = Clock::now();
	for (Node* node : tl_stack) {
		if (node == nullptr || node->closed) continue;
		node->end = now;
		node->duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - node->start).count();
		node->closed = true;
	}
}

inline void tl_flush_node(std::ostream& os, const Node* node, int indent) {
	if (node == nullptr) return;
	std::string pad((size_t)indent, ' ');
	os << pad << "- " << node->label << ": " << node->duration_ms << " ms\n";
	for (const auto& event : node->events) {
		if (event.kind == Node::Event::Kind::Attachment) {
			os << pad << "    - " << event.text << "\n";
		} else if (event.kind == Node::Event::Kind::Child) {
			tl_flush_node(os, event.child, indent + 4);
		}
	}
}

inline void tl_flush_to_file() {
	tl_force_close_open_nodes();
	std::error_code ec;
	auto out_path = std::filesystem::path(time_log_output_path);
	auto parent_path = out_path.parent_path();
	if (!parent_path.empty()) {
		std::filesystem::create_directories(parent_path, ec);
		if (ec) { std::cerr << "TimeLog: failed to create directory " << parent_path.string() << "\n"; return; }
	}
	std::ofstream ofs(time_log_output_path, std::ios::app);
	if (!ofs) { std::cerr << "TimeLog: failed to open " << time_log_output_path << "\n"; return; }

	// 获取当前时间并格式化为 年-月-日-时-分-秒
	auto now = std::chrono::system_clock::now();
	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	std::tm* tm_info = std::localtime(&now_time);
	char time_buf[32];
	std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d-%H-%M-%S", tm_info);
	ofs << "=== " << time_buf << " ===\n";
	double total = 0.0;
	for (const Node* root : tl_roots) { if (root != nullptr) total += root->duration_ms; }
	ofs << std::fixed << std::setprecision(3);
	ofs << "TimeLog summary (total " << total << " ms)\n";
	for (const Node* root : tl_roots) { tl_flush_node(ofs, root, 0); }
	ofs << '\n';
}

// 在程序退出时自动刷新日志
struct TimeLogFinalizer { ~TimeLogFinalizer() { tl_flush_to_file(); } };
inline TimeLogFinalizer time_log_finalizer;

// RAII 作用域对象
struct TimeLogScope {
	explicit TimeLogScope(const std::string &label) { tl_start(label); }
	explicit TimeLogScope(const char *label) { tl_start(label); }
	~TimeLogScope() { tl_end(); }
};

// 辅助宏生成唯一变量名
#define TL_CONCAT_IMPL(a,b) a##b
#define TL_CONCAT(a,b) TL_CONCAT_IMPL(a,b)

// 公共宏接口（非线程安全）
#define TL_SET_OUTPUT(path) ::tl::tl_set_output(path)
#define TL_START(label) ::tl::tl_start(label)
#define TL_END() ::tl::tl_end()
#define TL_ATTACH(info) ::tl::tl_attach(info)
#define TL_SCOPE(label) ::tl::TimeLogScope TL_CONCAT(_tl_scope_, __LINE__)(label)

} // namespace tl

#endif // TIME_LOG_DISABLE

