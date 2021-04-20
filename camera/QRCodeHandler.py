from abc import ABC


class QRCodeHandler(ABC):
    """
    An interface describing an object that can handle the content of a detected QR code
    """

    def handle(self, code_content: str):
        # TODO: Implement a code filter
        pass
