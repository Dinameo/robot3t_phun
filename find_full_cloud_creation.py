import json
import sys
sys.stdout.reconfigure(encoding='utf-8')

with open('C:/Users/nhan/.gemini/antigravity/brain/a14f4e23-e511-4b2b-9f43-f7597fe32959/.system_generated/logs/transcript.jsonl', 'r', encoding='utf-8') as f:
    for line in f:
        data = json.loads(line)
        content = str(data.get('content', ''))
        tool_calls = str(data.get('tool_calls', ''))
        if 'full_cloud_world' in content or 'full_cloud_world' in tool_calls:
            print(f"Step {data.get('step_index')}, Type {data.get('type')}")
            if data.get('tool_calls'):
                for tc in data['tool_calls']:
                    print("  Tool:", tc.get('name'))
                    args = tc.get('args', {})
                    for k, v in args.items():
                        if 'full_cloud' in str(v):
                            print(f"    {k}: {str(v)[:300]}")
