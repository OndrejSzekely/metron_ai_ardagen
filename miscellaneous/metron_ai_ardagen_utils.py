# This file is part of the Metron AI ArDaGen (https://github.com/OndrejSzekely/metron_ai_ardagen).
# Copyright (c) 2023 Ondrej Szekely.
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, version 3. This program
# is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details. You should have received a copy of the GNU General Public
# License along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
This file contains small domain-free functions specific to Metron AI ArDaGen.
"""

from os import path
import logging
from metron_shared import param_validators as param_val
from miscellaneous.custom_exceptions import OVExtNotLoaded


def load_ov_extension(ext_path: str, extension_name: str) -> None:
    """
    Loads <ext_path> extension. Has to be invoked after Isaac Sim app is running.

    Args:
        ext_path (str): Path to the extension's root folder.
        extension_name(str): Name of the extension.

    Raise:
        OVExtNotLoaded: Raised when OV can't load given extension.
    """
    param_val.check_type(ext_path, str)
    param_val.check_type(extension_name, str)
    param_val.check_folder_existence(ext_path)

    # Isaac Sim app has to be created before modules can be imported, so called in here.
    import omni.kit.app  # pylint: disable=import-outside-toplevel

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.add_path(path.join(ext_path, "exts"))
    ext_manager.set_extension_enabled_immediate(extension_name, True)

    if not ext_manager.is_extension_enabled(extension_name):
        raise OVExtNotLoaded(extension_name)

    logging.info("Extension `%s` enabled.", extension_name)
