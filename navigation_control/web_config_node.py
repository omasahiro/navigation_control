#!/usr/bin/env python3
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import uvicorn
from pydantic import BaseModel
from typing import List, Optional
from datetime import time
import json
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

class ScheduleEntry(BaseModel):
    section_id: str
    start_time: str
    end_time: str

class ScheduleConfig(BaseModel):
    schedule: List[ScheduleEntry]

class WebConfigNode(Node):
    def __init__(self):
        super().__init__('web_config_node')
        
        self.config_path = os.path.join(
            get_package_share_directory('navigation_control'),
            'config',
            'schedule.yaml'
        )
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Resumeサービスクライアントの作成
        self.resume_client = self.create_client(
            Trigger,
            'resume_navigation',
            callback_group=self.callback_group
        )
        
        self.app = FastAPI()
        self.setup_routes()
        
    def setup_routes(self):
        @self.app.get("/", response_class=HTMLResponse)
        async def get_index():
            return """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Navigation Schedule Configuration</title>
                <script src="https://cdn.tailwindcss.com"></script>
                <script>
                    let currentSchedule = [];

                    async function loadSchedule() {
                        const response = await fetch('/api/schedule');
                        const data = await response.json();
                        currentSchedule = data.schedule;
                        displaySchedule(data.schedule);
                    }

                    function displaySchedule(schedule) {
                        const tbody = document.getElementById('scheduleBody');
                        tbody.innerHTML = '';
                        
                        schedule.forEach((entry, index) => {
                            const row = document.createElement('tr');
                            row.innerHTML = `
                                <td class="border px-4 py-2">${entry.section_id}</td>
                                <td class="border px-4 py-2">${entry.start_time}</td>
                                <td class="border px-4 py-2">${entry.end_time}</td>
                                <td class="border px-4 py-2">
                                    <button onclick="editEntry(${index})" 
                                            class="bg-blue-500 text-white px-4 py-2 rounded">
                                        Edit
                                    </button>
                                    <button onclick="deleteEntry(${index})"
                                            class="bg-red-500 text-white px-4 py-2 rounded ml-2">
                                        Delete
                                    </button>
                                </td>
                            `;
                            tbody.appendChild(row);
                        });
                    }

                    async function saveSchedule() {
                        const form = document.getElementById('scheduleForm');
                        const formData = new FormData(form);
                        
                        const entry = {
                            section_id: formData.get('section_id'),
                            start_time: formData.get('start_time'),
                            end_time: formData.get('end_time')
                        };

                        const editIndex = form.getAttribute('data-edit-index');
                        let url = '/api/schedule';
                        let method = 'POST';

                        if (editIndex !== null && editIndex !== '') {
                            url = `/api/schedule/${editIndex}`;
                            method = 'PUT';
                        }

                        try {
                            const response = await fetch(url, {
                                method: method,
                                headers: {
                                    'Content-Type': 'application/json'
                                },
                                body: JSON.stringify(entry)
                            });

                            if (response.ok) {
                                await loadSchedule();
                                form.reset();
                                form.removeAttribute('data-edit-index');
                                document.getElementById('submitBtn').textContent = 'Add Schedule';
                            }
                        } catch (error) {
                            console.error('Error saving schedule:', error);
                        }
                    }

                    async function editEntry(index) {
                        const entry = currentSchedule[index];
                        const form = document.getElementById('scheduleForm');
                        
                        form.elements['section_id'].value = entry.section_id;
                        form.elements['start_time'].value = entry.start_time;
                        form.elements['end_time'].value = entry.end_time;
                        
                        form.setAttribute('data-edit-index', index);
                        document.getElementById('submitBtn').textContent = 'Update Schedule';
                    }

                    async function deleteEntry(index) {
                        if (confirm('Are you sure you want to delete this entry?')) {
                            try {
                                const response = await fetch(`/api/schedule/${index}`, {
                                    method: 'DELETE'
                                });

                                if (response.ok) {
                                    await loadSchedule();
                                }
                            } catch (error) {
                                console.error('Error deleting schedule:', error);
                            }
                        }
                    }

                    async function restartNavigation() {
                        try {
                            const response = await fetch('/api/restart', {
                                method: 'POST'
                            });
                            const result = await response.json();
                            alert(result.message);
                        } catch (error) {
                            console.error('Error restarting navigation:', error);
                            alert('Failed to restart navigation');
                        }
                    }
                </script>
            </head>
            <body class="bg-gray-100 p-8">
                <div class="max-w-4xl mx-auto">
                    <h1 class="text-2xl font-bold mb-6">Navigation Schedule Configuration</h1>
                    
                    <!-- Schedule Form -->
                    <div class="bg-white p-6 rounded-lg shadow-md mb-6">
                        <form id="scheduleForm" onsubmit="event.preventDefault(); saveSchedule();">
                            <div class="grid grid-cols-3 gap-4">
                                <div>
                                    <label class="block text-sm font-medium text-gray-700">Section ID</label>
                                    <input type="text" name="section_id" required
                                           class="mt-1 block w-full rounded-md border-gray-300 shadow-sm">
                                </div>
                                <div>
                                    <label class="block text-sm font-medium text-gray-700">Start Time</label>
                                    <input type="time" name="start_time" required
                                           class="mt-1 block w-full rounded-md border-gray-300 shadow-sm">
                                </div>
                                <div>
                                    <label class="block text-sm font-medium text-gray-700">End Time</label>
                                    <input type="time" name="end_time" required
                                           class="mt-1 block w-full rounded-md border-gray-300 shadow-sm">
                                </div>
                            </div>
                            <button type="submit" id="submitBtn"
                                    class="mt-4 bg-green-500 text-white px-4 py-2 rounded">
                                Add Schedule
                            </button>
                        </form>
                    </div>

                    <!-- Control Buttons -->
                    <div class="mb-6">
                        <button onclick="restartNavigation()"
                                class="bg-indigo-500 text-white px-4 py-2 rounded">
                            Restart Navigation
                        </button>
                    </div>

                    <!-- Schedule Table -->
                    <div class="bg-white rounded-lg shadow-md overflow-hidden">
                        <table class="min-w-full">
                            <thead>
                                <tr class="bg-gray-50">
                                    <th class="px-4 py-2 text-left">Section ID</th>
                                    <th class="px-4 py-2 text-left">Start Time</th>
                                    <th class="px-4 py-2 text-left">End Time</th>
                                    <th class="px-4 py-2 text-left">Actions</th>
                                </tr>
                            </thead>
                            <tbody id="scheduleBody">
                            </tbody>
                        </table>
                    </div>
                </div>
                <script>
                    loadSchedule();
                </script>
            </body>
            </html>
            """

        @self.app.get("/api/schedule")
        async def get_schedule():
            try:
                with open(self.config_path, 'r') as f:
                    return yaml.safe_load(f)
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        
        # web_config_node.pyの@self.app.post("/api/schedule")部分を修正
        @self.app.post("/api/schedule")
        async def add_schedule_entry(entry: ScheduleEntry):
            try:
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f)
        
                config['schedule'].append(entry.dict())
        
                with open(self.config_path, 'w') as f:
                    yaml.safe_dump(config, f)
        
                # Resume serviceを呼び出してナビゲーションを再開
                await self.call_resume_service()
        
                return {"status": "success"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        async def call_resume_service(self):
            """Resumeサービスを呼び出す"""
            if not self.resume_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Resume service not available')
                return

            request = Trigger.Request()
            try:
                response = await self.resume_client.call_async(request)
                if response.success:
                    self.get_logger().info('Navigation restarted successfully')
                else:
                    self.get_logger().warn(f'Failed to restart navigation: {response.message}')
            except Exception as e:
                self.get_logger().error(f'Error calling resume service: {str(e)}')

        @self.app.put("/api/schedule/{index}")
        async def update_schedule_entry(index: int, entry: ScheduleEntry):
            try:
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                if 0 <= index < len(config['schedule']):
                    config['schedule'][index] = entry.dict()
                    
                    with open(self.config_path, 'w') as f:
                        yaml.safe_dump(config, f)
                    
                    return {"status": "success"}
                else:
                    raise HTTPException(status_code=404, detail="Schedule entry not found")
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        @self.app.delete("/api/schedule/{index}")
        async def delete_schedule_entry(index: int):
            try:
                with open(self.config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                if 0 <= index < len(config['schedule']):
                    del config['schedule'][index]
                    
                    with open(self.config_path, 'w') as f:
                        yaml.safe_dump(config, f)
                    
                    return {"status": "success"}
                else:
                    raise HTTPException(status_code=404, detail="Schedule entry not found")
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        @self.app.post("/api/restart")
        async def restart_navigation():
            try:
                if not self.resume_client.wait_for_service(timeout_sec=1.0):
                    return {"status": "error", "message": "Navigation service not available"}

                request = Trigger.Request()
                future = self.resume_client.call_async(request)
                
                # ROS 2のコールバックを処理するためのスレッドを作成
                def process_callback():
                    while not future.done():
                        rclpy.spin_once(self)
                
                thread = threading.Thread(target=process_callback)
                thread.start()
                thread.join(timeout=5.0)  # 5秒でタイムアウト
                
                if future.done():
                    response = future.result()
                    if response.success:
                        return {"status": "success", "message": "Navigation restarted successfully"}
                    else:
                        return {"status": "error", "message": response.message}
                else:
                    return {"status": "error", "message": "Request timed out"}
                
            except Exception as e:
                return {"status": "error", "message": str(e)}

    def run(self, host="0.0.0.0", port=8000):
        # ROSのスピンを別スレッドで実行
        ros_thread = threading.Thread(target=lambda: rclpy.spin(self))
        ros_thread.daemon = True
        ros_thread.start()
        
        uvicorn.run(self.app, host=host, port=port)

def main(args=None):
    rclpy.init(args=args)
    node = WebConfigNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()