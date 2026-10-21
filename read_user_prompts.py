import json
import sys
sys.stdout.reconfigure(encoding='utf-8')

transcript_path = r'C:\Users\nhan\.gemini\antigravity\brain\a14f4e23-e511-4b2b-9f43-f7597fe32959\.system_generated\logs\transcript.jsonl'
with open(transcript_path, 'r', encoding='utf-8') as f:
    for line in f:
        data = json.loads(line)
        if data.get('source') == 'USER_EXPLICIT':
            print(f"Step {data.get('step_index')}:")
            print(data.get('content'))
            print('='*50)
