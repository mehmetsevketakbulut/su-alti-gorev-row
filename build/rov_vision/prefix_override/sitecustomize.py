import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/akintay/Desktop/su-alti-gorev-row/install/rov_vision'
