class FileUtils:
    """
    Contains utilities related to file system operations.
    """

    @staticmethod
    def open(filePath, permArgs):
        """
        Wraps the open command. To ease testing, all open() calls should be done
        using this method, rather than invoking open() directly.
        """
        return open(filePath, permArgs)
