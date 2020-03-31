import ecto
import sys

HAS_GUI=False

try:
    from PySide.QtCore import QTimer, Qt
    from PySide.QtGui import QWidget, QDialog, QPushButton, QApplication, QCheckBox, \
     QVBoxLayout, QLabel, QHBoxLayout, QLineEdit, QScrollArea, QMainWindow, QSizePolicy, \
     QSpacerItem
    HAS_GUI=True
    class TendrilWidget(QWidget):
        def __init__(self, name, tendril, parent=None):
            super(TendrilWidget,self).__init__(parent)
            hlayout = QHBoxLayout(self)
            label = QLabel("&" + name)
            hlayout.addWidget(label)
            self.thunker = TendrilThunker(tendril)
            if tendril.val == True or tendril.val == False:
                spacer = QSpacerItem(0, 0, hPolicy=QSizePolicy.Expanding, vPolicy=QSizePolicy.Minimum)
                hlayout.addItem(spacer)
                checkbox = QCheckBox(self)
                checkbox.setCheckState(Qt.Checked if tendril.val else Qt.Unchecked)
                checkbox.stateChanged.connect(self.thunker.update)
                label.setBuddy(checkbox)
                hlayout.addWidget(checkbox)
            else:
                edit = QLineEdit(str(tendril.val), self)
                edit.textChanged.connect(self.thunker.update)
                label.setBuddy(edit)
                hlayout.addWidget(edit)
            self.setLayout(hlayout)
            self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

    class DynamicReconfigureForm(QWidget):
        def __init__(self, plasm, whitelist=None, parent=None):
            '''
            Accepts a plasm for connecting tendrils to a frontend widget. Also
            takes an optional whitelist parameter so that you can selectively
            decide which parameters are for show.

            :param Plasm ecto.Plasm: live squelchy thing that contains the configuration of your system locked in its parameter tendrils.
            :param whitelist dic: dictionary of {cell name: [tendril_name]} to show
            '''
            super(DynamicReconfigureForm, self).__init__(parent)
            self.plasm = plasm
            self.whitelist = whitelist
            self.setWindowTitle("Dynamic Reconfigure")
            plasm.configure_all()
            self.generate_dialogs()
            self.sched = ecto.Scheduler(plasm)

        def exec_one(self):
            if not self.sched.running():
                self.sched.prepare_jobs()
            try:
                #give ecto a slice of time
                rval = self.sched.run(1000) #exec for 1000 microseconds
                if not rval: #quit condition
                    sys.exit(1)
            except KeyboardInterrupt as e: #ctrl-c
                sys.exit(1)

        def is_in_whitelist(self, cell_name, tendril_name):
            if cell_name in self.whitelist.keys():
                if tendril_name in self.whitelist[cell_name]:
                    return True
            return False

        def generate_dialogs(self):
            vlayout = QVBoxLayout()
            commit_button = QPushButton("&Commit")
            w_all = QWidget()
            v = QVBoxLayout()
            for cell in self.plasm.cells():
                tendril_widgets = []
                for name, tendril in cell.params:
                    if self.whitelist is None or self.is_in_whitelist(cell.name(), name):
                        tw = TendrilWidget(name, tendril)
                        commit_button.clicked.connect(tw.thunker.commit)
                        tendril_widgets.append(tw)
                if tendril_widgets:
                    w = QWidget()
                    cvlayout = QVBoxLayout()
                    cvlayout.addWidget(QLabel(cell.name()))
                    for tendril_widget in tendril_widgets:
                        cvlayout.addWidget(tendril_widget)
                    w.setLayout(cvlayout)
                    # w.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
                    v.addWidget(w)
            w_all.setLayout(v)
            s = QScrollArea()
            s.setWidget(w_all)
            vlayout.addWidget(s)
            vlayout.addWidget(commit_button)
            self.setLayout(vlayout)


except ImportError as e:
    print e
    pass


class TendrilThunker(object):

    def __init__(self, tendril):
        self.tendril = tendril
        self.val = None

    def update(self, val):
        if type(self.tendril.val) is bool:
            self.val = True if (val == 2) else False
        else:
            self.val = val

    def commit(self):
        if self.val is not None:
            x = type(self.tendril.val)(self.val)
            self.tendril.set(x)
            self.val = None

def gui_execute(plasm):
    if HAS_GUI:
        # Create a Qt application
        app = QApplication(sys.argv)
        # Create and show the form
        form = DynamicReconfigureForm(plasm)
        timer = QTimer(form)
        timer.timeout.connect(form.exec_one)
        timer.start()
        form.show()
        # Run the main Qt loop
        return app.exec_()

    else:
        print >>sys.stderr, "Could not import PySide. Please install python-pyside."
        sys.exit(1)
