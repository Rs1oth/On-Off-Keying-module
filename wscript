# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

def configure(conf):
    conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('on-off-keying-module', ['core','network', 'mpi'])
    module.source = [
        'helper/on-off-keying-module-helper.cc',
        'model/on-off-keying-channel.cc',
        'model/on-off-keying-net-device.cc',
        'model/on-off-keying-remote-channel.cc',
        'model/OOK-header.cc',
        'model/OOK-error-model.cc',
        'model/OOK-error-model-2Interference.cc',
        'model/PAM-error-model.cc',
        'model/vlc-propagation-loss-model.cc',
        'model/VLC-Mobility-Model.cc',
        ]

    module_test = bld.create_ns3_module_test_library('on-off-keying-module')
    module_test.source = [
        'test/on-off-keying-module-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'on-off-keying-module'
    headers.source = [
        'helper/on-off-keying-module-helper.h',
        'model/on-off-keying-net-device.h',
        'model/on-off-keying-channel.h',
        'model/on-off-keying-remote-channel.h',
        'model/OOK-header.h',
        'model/OOK-error-model.h',
        'model/OOK-error-model-2Interference.h',
        'model/PAM-error-model.h',
        'model/vlc-propagation-loss-model.h',
        'model/VLC-Mobility-Model.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')
    
    #bld.ns3_python_bindings()
