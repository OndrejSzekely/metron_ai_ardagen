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
