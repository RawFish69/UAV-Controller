import subprocess

gazebo_sdf = """<?xml version='1.0'?><sdf version='1.8'><model name='test_marker'><static>true</static><pose>0 0 5 0 0 0</pose><link name='link'><visual name='visual'><geometry><sphere><radius>1.0</radius></sphere></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"""

def escape(text):
    return text.replace('\\', '\\\\').replace('"', '\\"')

req = f'sdf: "{escape(gazebo_sdf)}"'

cmd = [
    'gz', 'service', '-s', '/world/uav_flat_world/create',
    '--reqtype', 'gz.msgs.EntityFactory',
    '--reptype', 'gz.msgs.Boolean',
    '--timeout', '5000',
    '--req', req
]

print("Running command:", " ".join(cmd))
res = subprocess.run(cmd, capture_output=True, text=True)
print("RC:", res.returncode)
print("STDOUT:", res.stdout)
print("STDERR:", res.stderr)
