import os
Import('env')
from building import *

cwd = GetCurrentDir()
objs = []

env.Append(CPPDEFINES = [
    'STM32G431xx',
    'FOC_TARGET_STM32G431',
])

for item in os.listdir(cwd):
    path = os.path.join(cwd, item, 'SConscript')
    if os.path.isfile(path):
        objs += SConscript(path)

Return('objs')
