# 选手运行demo命令

## 命令格式
```bash
bash run_and_test.sh <solution_file> <input_file> <output_file> <timeout_ms>
```
其中:
1. solution_file：示例程序，位于demos目录下，有c、c++、java和python四种语言  
2. input_file: 用例数据, 位于data目录下，有多组测试用例，命名格式为：offical_*.in  
3. output_file: 示例程序输出结果文件
4. timeout_ms: 用例运行超时时间


## 执行如下命令可运行此demo程序
```
bash run_and_test.sh ./demos/Solution.cpp ./data/official_1.in ./data/official_1.out 60000
```

## 执行如下命令可运行判题器程序
chmod +x Checker
./Checker <input_file> <output_file> <answer_file>
