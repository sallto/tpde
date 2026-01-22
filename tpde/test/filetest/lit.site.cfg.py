# SPDX-FileCopyrightText: 2025 Contributors to TPDE <https://tpde.org>
#
# SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

import os

config.tpde_src_root = r'/home/tom/Code/tpde2/tpde'
config.tpde_obj_root = r'/home/tom/Code/tpde2/tpde'

config.llvm_tools_dir = r'/usr/lib64/llvm20/bin'

import lit.llvm
lit.llvm.initialize(lit_config, config)

lit_config.load_config(config, os.path.join(config.tpde_src_root, 'test/filetest/lit.cfg.py'))
