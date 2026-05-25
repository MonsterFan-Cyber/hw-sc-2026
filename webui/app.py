#!/usr/bin/env python3
import json
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import re

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from web.parser.binary_parser import BinaryParser, ParseError
from web.parser.data_validator import DataValidator, ValidationError
from web.utils.logger import Logger


class SnapshotViewerHandler(SimpleHTTPRequestHandler):
    parsed_data = None
    logger = Logger("WebApp")
    
    def __init__(self, *args, **kwargs):
        self.parser = BinaryParser(self.logger)
        self.validator = DataValidator(self.logger)
        super().__init__(*args, directory=os.path.join(os.path.dirname(__file__), 'static'), **kwargs)
    
    def do_GET(self):
        parsed_path = urlparse(self.path)
        
        if parsed_path.path == '/' or parsed_path.path == '/index.html':
            self.serve_index()
        elif parsed_path.path == '/metadata':
            self.handle_get_metadata()
        elif parsed_path.path.startswith('/frame/'):
            self.handle_get_frame(parsed_path.path)
        else:
            super().do_GET()
    
    def do_POST(self):
        parsed_path = urlparse(self.path)
        
        if parsed_path.path == '/upload':
            self.handle_file_upload()
        else:
            self.send_error(404, "Not Found")
    
    def serve_index(self):
        index_path = os.path.join(os.path.dirname(__file__), 'static', 'index.html')
        if os.path.exists(index_path):
            with open(index_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        else:
            self.send_error(404, "index.html not found")
    
    def handle_file_upload(self):
        try:
            content_type = self.headers.get('Content-Type', '')
            if not content_type.startswith('multipart/form-data'):
                self.send_json_response({'error': 'Invalid content type'}, 400)
                return
            
            boundary_match = re.search(r'boundary=([^\s;]+)', content_type)
            if not boundary_match:
                self.send_json_response({'error': 'No boundary found'}, 400)
                return
            
            boundary = boundary_match.group(1).strip()
            content_length = int(self.headers.get('Content-Length', 0))
            
            body = self.rfile.read(content_length)
            
            boundary_marker = f'--{boundary}'.encode()
            
            filename = None
            file_data = None
            
            start = 0
            while True:
                boundary_pos = body.find(boundary_marker, start)
                if boundary_pos == -1:
                    break
                
                part_start = boundary_pos + len(boundary_marker)
                next_boundary = body.find(boundary_marker, part_start)
                
                if next_boundary == -1:
                    part = body[part_start:]
                else:
                    part = body[part_start:next_boundary]
                
                while part and part[0:1] in (b'\r', b'\n'):
                    part = part[1:]
                
                while part and part[-1:] in (b'\r', b'\n'):
                    part = part[:-1]
                
                if not part or part == b'--':
                    start = next_boundary
                    continue
                
                header_end = part.find(b'\r\n\r\n')
                if header_end == -1:
                    header_end = part.find(b'\n\n')
                    if header_end == -1:
                        start = next_boundary
                        continue
                    header_sep_len = 2
                else:
                    header_sep_len = 4
                
                headers_part = part[:header_end].decode('utf-8', errors='ignore')
                data_part = part[header_end + header_sep_len:]
                
                name_match = re.search(r'name="([^"]+)"', headers_part)
                if not name_match:
                    start = next_boundary
                    continue
                
                field_name = name_match.group(1)
                
                if field_name == 'file':
                    filename_match = re.search(r'filename="([^"]+)"', headers_part)
                    if filename_match:
                        filename = filename_match.group(1)
                        file_data = data_part
                        break
                
                start = next_boundary
            
            if not filename or file_data is None:
                self.send_json_response({'error': 'No file uploaded'}, 400)
                return
            
            if not filename.endswith('.snpst'):
                self.send_json_response({'error': 'Invalid file extension. Must be .snpst'}, 400)
                return
            
            temp_path = os.path.join('/tmp', 'temp_snapshot.snpst')
            with open(temp_path, 'wb') as f:
                f.write(file_data)
            
            self.logger.info(f"File uploaded: {filename}")
            
            parsed_data = self.parser.parse_file(temp_path)
            SnapshotViewerHandler.parsed_data = parsed_data
            
            os.remove(temp_path)
            
            metadata = {
                'filename': filename,
                'polygon_count': parsed_data['metadata']['polygon_count'],
                'snapshot_count': parsed_data['metadata']['snapshot_count'],
                'feasible_region_vertices': parsed_data['metadata']['feasible_region_vertices']
            }
            
            self.logger.info(f"File parsed successfully: {metadata}")
            self.send_json_response({'success': True, 'metadata': metadata})
            
        except ParseError as e:
            self.logger.error(f"Parse error: {str(e)}")
            self.send_json_response({'error': f'Parse error: {str(e)}'}, 400)
        except ValidationError as e:
            self.logger.error(f"Validation error: {str(e)}")
            self.send_json_response({'error': f'Validation error: {str(e)}'}, 400)
        except Exception as e:
            self.logger.error(f"Unexpected error: {str(e)}")
            import traceback
            self.logger.error(traceback.format_exc())
            self.send_json_response({'error': f'Unexpected error: {str(e)}'}, 500)
    
    def handle_get_metadata(self):
        if SnapshotViewerHandler.parsed_data is None:
            self.send_json_response({'error': 'No file loaded'}, 404)
            return
        
        metadata = SnapshotViewerHandler.parsed_data['metadata']
        
        feasible_region = SnapshotViewerHandler.parsed_data['feasible_region']
        feasible_region_data = {
            'vertices': list(feasible_region.vertices)
        }
        
        illegal_frames = []
        for snapshot in SnapshotViewerHandler.parsed_data['snapshots']:
            illegal_ids = snapshot.get_illegal_polygon_ids()
            if illegal_ids:
                illegal_frames.append({
                    'frame_id': snapshot.frame_id,
                    'illegal_polygon_ids': illegal_ids
                })
        
        response = {
            'metadata': metadata,
            'feasible_region': feasible_region_data,
            'illegal_frames': illegal_frames
        }
        
        self.send_json_response(response)
    
    def handle_get_frame(self, path):
        if SnapshotViewerHandler.parsed_data is None:
            self.send_json_response({'error': 'No file loaded'}, 404)
            return
        
        try:
            frame_id = int(path.split('/frame/')[1].split('?')[0])
        except (ValueError, IndexError):
            self.send_json_response({'error': 'Invalid frame ID'}, 400)
            return
        
        snapshots = SnapshotViewerHandler.parsed_data['snapshots']
        if frame_id < 0 or frame_id >= len(snapshots):
            self.send_json_response({'error': f'Frame ID {frame_id} out of range'}, 400)
            return
        
        snapshot = snapshots[frame_id]
        polygons = SnapshotViewerHandler.parsed_data['polygons']
        
        polygon_states = []
        for i, polygon in enumerate(polygons):
            translation, legality = snapshot.get_polygon_state(i)
            vertices = polygon.get_translated_vertices(translation[0], translation[1])
            
            polygon_states.append({
                'id': polygon.polygon_id,
                'vertices': vertices,
                'is_legal': legality == 1,
                'translation': list(translation)
            })
        
        feasible_region = SnapshotViewerHandler.parsed_data['feasible_region']
        
        response = {
            'frame_id': frame_id,
            'total_frames': len(snapshots),
            'polygon_states': polygon_states,
            'feasible_region': {
                'vertices': list(feasible_region.vertices)
            },
            'legal_count': snapshot.get_legal_count(),
            'illegal_count': snapshot.get_illegal_count(),
            'illegal_polygon_ids': snapshot.get_illegal_polygon_ids()
        }
        
        self.send_json_response(response)
    
    def send_json_response(self, data, status=200):
        content = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', len(content))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(content)
    
    def log_message(self, format, *args):
        self.logger.info(f"{self.address_string()} - {format % args}")


def main():
    import socket
    
    port = 8000
    server_address = ('', port)
    
    class ReuseAddrHTTPServer(HTTPServer):
        allow_reuse_address = True
    
    httpd = ReuseAddrHTTPServer(server_address, SnapshotViewerHandler)
    
    print("Web 快照播放器服务器启动")
    print(f"访问地址: http://localhost:{port}")
    print("按 Ctrl+C 停止服务器")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n服务器已停止")
        httpd.server_close()


if __name__ == '__main__':
    main()
