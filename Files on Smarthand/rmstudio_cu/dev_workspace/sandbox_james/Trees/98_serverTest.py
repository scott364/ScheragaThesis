import sys
print("Python version")
print (sys.version)
print("Version info.")
print (sys.version_info)

from network_data import FT_Broadcaster

caster = FT_Broadcaster( alpha = 0.012, beta = 0.035 )
caster.prime_optoforce()
caster.broadcast()