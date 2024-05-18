from PySide6.QtWidgets import QWidget, QDoubleSpinBox, QAbstractSpinBox

class CustomDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, parent:QWidget, decimals:int, step:float, minRange:float, maxRange:float, suffix=""):
        super().__init__(parent)

        self.setDecimals(decimals)
        self.setSingleStep(step)
        self.setRange(minRange, maxRange)
        self.setSuffix(suffix)
        self.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.setFrame(False)
