import re

with open('dobot_cr10a/Assets/Scenes/SampleScene.unity', 'r', encoding='utf-8') as f:
    text = f.read()

transforms = {}
gameobjects = {}
for m in re.finditer(r'--- !u!1 &(\d+)\nGameObject:\n  m_ObjectHideFlags: 0\n[\s\S]*?m_Name: ([^\n]+)', text):
    go_id = m.group(1)
    name = m.group(2)
    gameobjects[go_id] = name

for m in re.finditer(r'--- !u!4 &(\d+)\nTransform:\n[\s\S]*?m_GameObject: \{fileID: (\d+)\}[\s\S]*?m_LocalRotation: \{x: ([^,]+), y: ([^,]+), z: ([^,]+), w: ([^\}]+)\}\n  m_LocalPosition: \{x: ([^,]+), y: ([^,]+), z: ([^\}]+)\}\n  m_LocalScale: \{x: ([^,]+), y: ([^,]+), z: ([^\}]+)\}[\s\S]*?m_Children:([\s\S]*?)m_Father: \{fileID: ([^\}]+)\}', text):
    t_id = m.group(1)
    go_id = m.group(2)
    name = gameobjects.get(go_id, 'Unknown')
    rot = [float(x) for x in m.groups()[2:6]]
    pos = [float(x) for x in m.groups()[6:9]]
    scale = [float(x) for x in m.groups()[9:12]]
    father = m.group(13)
    transforms[t_id] = {'name': name, 'go_id': go_id, 'rot': rot, 'pos': pos, 'scale': scale, 'father': father}

for tid, t in transforms.items():
    if any(k in t['name'] for k in ['axis', 'Global', 'Camera', 'Garment', 'manual', 'dobot']):
        print(f"{t['name']:20s} (id={tid}): pos={t['pos']}, rot={t['rot']}, father={t['father']}")
