import pyvisa

rm = pyvisa.ResourceManager()
print(f'Connected VISA resources: {rm.list_resources()}')

value = -23.4 #for now example

fieldnames = [
    'Set Kp',
    'Set Ki',
    'Set Kd',
    'Set Position',
    'Emergency Stop',
    'Allow motion',
    'Go to Position 0'
]

commands = {
    f'#k{value}%',
    f'${value}%',
    f'^{value}%',
    f'*{value}%',
    '!s%',
    '@d#',
    '-'
}