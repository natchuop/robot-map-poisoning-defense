import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/natch/projects/robot-map-poisoning-defense/install/attack_node'
