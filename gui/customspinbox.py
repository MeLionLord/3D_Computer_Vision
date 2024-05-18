from PySide6.QtWidgets import QWidget, QSpinBox, QAbstractSpinBox

class CustomSpinBox(QSpinBox):
    def __init__(self, parent:QWidget, step:int, minRange:int, maxRange:int, suffix=""):
        super().__init__(parent)

        self.setSingleStep(step)
        self.setRange(minRange, maxRange)
        self.setSuffix(suffix)
        self.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.setFrame(False)
