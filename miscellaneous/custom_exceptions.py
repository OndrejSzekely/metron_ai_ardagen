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
This file defines custom exceptions.
"""


from metron_shared import param_validators as shared_param_val


class OVExtNotLoaded(IOError):
    """
    Omniverse Extensions was not loaded.

    Attributes:
        message (str): Exception message.
    """

    def __init__(self, ext_name: str):
        """

        Args:
            ext_name (str): OV Extension name.
        """
        shared_param_val.check_type(ext_name, str)

        self.message = f"Omniverse Extension `{ext_name}` can't be loaded."

        super().__init__(self.message)

    def __str__(self) -> str:
        """
        Used to get string representation of the error.

        Returns (str): String error representation.

        """
        return self.message
