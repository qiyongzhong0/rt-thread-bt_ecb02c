from building import *

cwd = GetCurrentDir()
path = [cwd+'/inc']
src  = Glob('src/*.c')
 
group = DefineGroup('bt_ecb02c', src, depend = ['PKG_USING_BT_ECB02C'], CPPPATH = path)

Return('group')