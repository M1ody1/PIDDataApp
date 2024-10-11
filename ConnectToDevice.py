import pyvisa

rm = pyvisa.ResourceManager()
print(f'Connected VISA resources: {rm.list_resources()}')

