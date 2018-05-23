# -*- coding: utf-8 -*-
from datetime import datetime

def waspframe_parse(waspframe_string):
    """Libelium Waspmote WaspFrame string converter to dict
    """
    if waspframe_string[:3] != '<=>' and waspframe_string[-1:] != '#':
        raise RuntimeError('Given string is not Waspmote ASCII Data Frame')
    elif ord(waspframe_string[3]) != 0x86:
        raise RuntimeError('Only Information frame for Waspmote v15 supported')

    waspframe_dict = dict()
    frame_fields = waspframe_string.split('#')
    is_float = lambda x: x.replace('.','',1).isdigit() and "." in x
    for fidx, field in enumerate(frame_fields):
        if ':' in field:
            lhv, rhv = field.split(':', 1)
            if lhv == 'GMT':
                waspframe_dict[lhv] = datetime.strptime(rhv, '%a, %y/%m/%d, %H:%M:%S')
            else:
                waspframe_dict[lhv] = rhv if not is_float(rhv) else float(rhv)
        elif fidx == 1: # Serial ID unique for every Waspmote board
            waspframe_dict['serial_id'] = field
        elif fidx == 2: # Waspmote ID set in firmware
            waspframe_dict['waspmote_id'] = field
        elif fidx == 3: # Frame Sequence
            waspframe_dict['frame_sequence'] = field
    return waspframe_dict
