import matplotlib.pyplot as plt
from weakref import ref


class RealTimeGraph:
    def __init__(self, parent):
        self.parent = ref(parent)
        self.name = f'{self.parent().name}'
        self.open = False
        self.fig, self.ax = self._init_figure()

    def _init_figure(self):
        fig, ax = plt.subplots(2, 1)
        fig.suptitle(f'{self.name}')
        ax[0].plot(0, 0)
        ax[0].set_title("Position")
        ax[1].plot(0, 0)
        ax[1].set_title("Voltage")
        fig.tight_layout()
        plt.close(fig)
        return fig, ax

    def _update(self):
        x = [i - self.parent().parent().parent().start_time for i in self.parent().diagnose_times]
        # Update the position plot
        self.ax[0].cla()  # Clear plot before plotting new data
        self.ax[0].plot(x, self.parent().position_history, color='blue')

        # Update the voltage plot
        self.ax[1].cla()  # Clear plot before plotting new data
        raw_voltage = [i[0] for i in self.parent().voltage_history]
        filtered_voltage = [i[1] for i in self.parent().voltage_history]

        self.ax[1].plot(x, raw_voltage, color='orange')
        self.ax[1].plot(x, filtered_voltage, color='red')

        # Annotate each point at the graph with the current state of the leg
        for i, txt in enumerate(self.parent().state_history):
            self.ax[0].annotate(txt, (x[i], self.parent().position_history[i]), color=self.ax[0].lines[0].get_color())
            self.ax[1].annotate(txt, (x[i], raw_voltage[i]), color=self.ax[1].lines[0].get_color())
            self.ax[1].annotate(txt, (x[i], filtered_voltage[i]), color=self.ax[1].lines[1].get_color())

    def _open_figure(self):
        """
        Method that allows to reopen a closed figure.
        :return: None
        """
        new_manager = None

        if not self.open:
            dummy = plt.figure()
            new_manager = dummy.canvas.manager
            new_manager.canvas.figure = self.fig
            self.open = True

        if new_manager is not None:
            self.fig.set_canvas(new_manager.canvas)

    def close_figure(self):
        plt.close(self.fig)
        self.open = False

    def plot_realtime(self, duration=0.01):
        """
        Method that shows the figure.
        :param duration: How long is the figure displayed. Basically a refresh rate.
        :return: None
        """
        self._open_figure()
        self._update()
        plt.pause(duration)
        