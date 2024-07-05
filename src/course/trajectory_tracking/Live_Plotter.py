#!/usr/bin/python3

import tkinter as tk
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.backends.tkagg as tkagg
from matplotlib.backends.backend_agg import FigureCanvasAgg
import pygame

class Dynamic2DFigure():
    def __init__(self,
                 figsize=(8,8),
                 edgecolor="black",
                 rect=[0.1, 0.1, 0.8, 0.8],
                 *args, **kwargs):
        self.graphs = {}
        self.texts = {}
        self.fig = plt.Figure(figsize=figsize, edgecolor=edgecolor)
        self.ax = self.fig.add_axes(rect)
        self.fig.tight_layout()
        self.marker_text_offset_x = 0
        self.marker_text_offset_y = 0
        if kwargs["title"] is not None:
            self.fig.suptitle(kwargs["title"])
        self.axis_equal = False
        self.invert_xaxis = False
        self.invert_yaxis = False

    def set_invert_x_axis(self):
        self.invert_xaxis = True

    def set_invert_y_axis(self):
        self.invert_yaxis = True

    def set_axis_equal(self):
        self.axis_equal = True

    def add_graph(self, name, label="", window_size=10, x0=None, y0=None,
                  linestyle='-', linewidth=1, marker="", color="k",
                  markertext=None, marker_text_offset_x=0,marker_text_offset_y=0):
        self.marker_text_offset_x = marker_text_offset_x
        self.marker_text_offset_y = marker_text_offset_y

        if x0 is None or y0 is None:
            x0 = np.zeros(window_size)
            y0 = np.zeros(window_size)
            new_graph, = self.ax.plot(x0, y0, label=label,
                                      linestyle=linestyle, linewidth=linewidth,
                                      marker=marker, color=color)
            if markertext is not None:
                new_text = self.ax.text(x0[-1] + marker_text_offset_x, y0[-1] + marker_text_offset_y,
                                         markertext)
        else:
            new_graph, = self.ax.plot(x0, y0, label=label,
                                      linestyle=linestyle, linewidth=linewidth,
                                      marker=marker, color=color)
            if markertext is not None:
                new_text = self.ax.text(x0[-1] + marker_text_offset_x, y0[-1] + marker_text_offset_y,
                                         markertext)

        self.graphs[name]           = new_graph
        if markertext is not None:
            self.texts[name + "_TEXT"] = new_text

    def roll(self, name, new_x, new_y):
        graph = self.graphs[name]
        if graph is not None:
            x, y = graph.get_data()
            x = np.roll(x, -1)
            x[-1] = new_x
            y = np.roll(y, -1)
            y[-1] = new_y
            graph.set_data((x, y))
            self.rescale()
        if name + "_TEXT" in self.texts:
            graph_text = self.texts[name + "_TEXT"]
            x = new_x + self.marker_text_offset_x
            y = new_y + self.marker_text_offset_y
            graph_text.set_position((x, y))
            self.rescale()

    def update(self, name, new_x_vec, new_y_vec, new_colour='k'):
        graph = self.graphs[name]
        if graph is not None:
            graph.set_data((np.array(new_x_vec), np.array(new_y_vec)))
            graph.set_color(new_colour)
            self.rescale()
        if name + "_TEXT" in self.texts:
            graph_text = self.texts[name + "_TEXT"]
            x = new_x_vec[-1] + self.marker_text_offset_x
            y = new_y_vec[-1] + self.marker_text_offset_y
            graph_text.set_position((x, y))
            self.rescale()

    def rescale(self):
        xmin = float("inf")
        xmax = -1*float("inf")
        ymin, ymax = self.ax.get_ylim()
        for name, graph in self.graphs.items():
            xvals, yvals = graph.get_data()
            xmin_data = xvals.min()
            xmax_data = xvals.max()
            ymin_data = yvals.min()
            ymax_data = yvals.max()
            xmin_padded = xmin_data-0.05*(xmax_data-xmin_data)
            xmax_padded = xmax_data+0.05*(xmax_data-xmin_data)
            ymin_padded = ymin_data-0.05*(ymax_data-ymin_data)
            ymax_padded = ymax_data+0.05*(ymax_data-ymin_data)
            xmin = min(xmin_padded, xmin)
            xmax = max(xmax_padded, xmax)
            ymin = min(ymin_padded, ymin)
            ymax = max(ymax_padded, ymax)
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        if self.axis_equal:
            self.ax.set_aspect('equal')
        if self.invert_xaxis:
            self.ax.invert_xaxis()
        if self.invert_yaxis:
            self.ax.invert_yaxis()


class DynamicFigure():
    def __init__(self, *args, **kwargs):
        self.graphs = {}
        self.fig = plt.Figure(figsize=(3, 2), edgecolor="black")
        self.ax = self.fig.add_axes([0.2, 0.2, 0.6, 0.6])
        self.fig.tight_layout()
        if kwargs["title"] is not None:
            self.fig.suptitle(kwargs["title"])

    def add_graph(self, name, label="", window_size=10, x0=None, y0=None,
                  linestyle='-', linewidth=1, marker="", color="k"):
        if x0 is None or y0 is None:
            x0 = np.zeros(window_size)
            y0 = np.zeros(window_size)
            new_graph, = self.ax.plot(x0, y0, label=label,
                                      linestyle=linestyle, linewidth=linewidth, color=color)
        else:
            new_graph, = self.ax.plot(x0, y0, label=label,
                                      linestyle=linestyle, linewidth=linewidth, color=color)
        self.graphs[name]           = new_graph

    def roll(self, name, new_x, new_y):
        graph = self.graphs[name]
        if graph is not None:
            x, y = graph.get_data()
            x = np.roll(x, -1)
            x[-1] = new_x
            y = np.roll(y, -1)
            y[-1] = new_y
            graph.set_data((x, y))
            self.rescale()

    def rescale(self):
        xmin = float("inf")
        xmax = -1*float("inf")
        ymin, ymax = self.ax.get_ylim()
        for name, graph in self.graphs.items():
            xvals, yvals = graph.get_data()
            xmin_data = xvals.min()
            xmax_data = xvals.max()
            ymin_data = yvals.min()
            ymax_data = yvals.max()
            xmin_padded = xmin_data-0.05*(xmax_data-xmin_data)
            xmax_padded = xmax_data+0.05*(xmax_data-xmin_data)
            ymin_padded = ymin_data-0.05*(ymax_data-ymin_data)
            ymax_padded = ymax_data+0.05*(ymax_data-ymin_data)
            xmin = min(xmin_padded, xmin)
            xmax = max(xmax_padded, xmax)
            ymin = min(ymin_padded, ymin)
            ymax = max(ymax_padded, ymax)
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)


class LivePlotter():
    def __init__(self, tk_title=None, win_pos=[None,None]):
        self._default_w = 150
        self._default_h = 100
        self._graph_w = 0
        self._graph_h = 0
        self._surf_w = 0
        self._surf_h = 0

        self._figs = []
        self._fcas = {}
        self._photos = {}

        self._text_id = None
        self._empty = True

        self._root = tk.Tk()
        self._root.geometry('+{}+{}'.format(win_pos[0],win_pos[1])) # Set window position (w.r.t. top-left corner)
        if tk_title is not None:
            self._root.title(tk_title)

        self._canvas = tk.Canvas(self._root, width=self._default_w, height=self._default_h)
        self._canvas.config(bg="#6A6A6A")
        self._text_id = self._canvas.create_text(
            (self._default_w/2, self._default_h/2),
            text="No live plots\ncreated yet.")
        self._canvas.grid(row=0, column=0)

        self._display = None
        self._game_frame = None
        self._pygame_init = False

        self._surfs = []
        self._surf_coords = {}

    def plot_figure(self, fig):
        if self._empty:
            self._empty = False
            self._canvas.delete(self._text_id)

        f_w = fig.get_window_extent().width
        f_h = fig.get_window_extent().height
        f_w, f_h = int(f_w), int(f_h)

        # draw out figure
        fca = FigureCanvasAgg(fig)
        fca.draw()

        f_w, f_h = fca.get_renderer().get_canvas_width_height()
        f_w, f_h = int(f_w), int(f_h)

        self._graph_h += f_h
        self._graph_w = max(self._graph_w, f_w)
        self._canvas.config(width=self._graph_w, height=self._graph_h)
        self._canvas.grid(row=0, column=0)

        photo = tk.PhotoImage(master=self._canvas, width=f_w, height=f_h)
        self._canvas.create_image(f_w/2, self._graph_h-f_h/2, image=photo)
        tkagg.blit(photo, fca.get_renderer()._renderer, colormode=2)
        self._root.update()

        self._figs.append(fig)
        self._fcas[fig] = fca
        self._photos[fig] = photo

    def plot_new_figure(self):
        fig = plt.Figure(figsize=(3, 2), edgecolor="black")
        ax = fig.add_axes([0.2, 0.2, 0.6, 0.6])
        fig.tight_layout()
        # this stores the figure locally as well
        self.plot_figure(fig)
        return fig, ax

    def plot_new_dynamic_figure(self, title="", **kwargs):
        dyfig = DynamicFigure(title=title, **kwargs)
        fig = dyfig.fig
        # this stores the figure locally as well
        self.plot_figure(fig)
        return dyfig

    def plot_new_dynamic_2d_figure(self, title="", **kwargs):
        dy2dfig = Dynamic2DFigure(title=title, **kwargs)
        fig = dy2dfig.fig
        # this stores the figure locally as well
        self.plot_figure(fig)
        return dy2dfig

    def refresh_figure(self, fig):
        self._fcas[fig].draw()
        self._fcas[fig].flush_events()
        fig.canvas.draw()
        fig.canvas.flush_events()
        tkagg.blit(
            self._photos[fig],
            self._fcas[fig].get_renderer()._renderer,
            colormode=2)
        self._root.update()

    def init_pygame(self):
        self._game_frame = tk.Frame(
            self._root,
            width=self._surf_w,
            height=self._surf_h)
        self._game_frame.grid(row=0, column=1)

        os.environ['SDL_WINDOWID'] = str(self._game_frame.winfo_id())
        self._game_frame.update()
        pygame.display.init()

    def plot_surface(self, surf):
        s_w, s_h = surf.get_size()

        self._surf_w += s_w
        self._surf_h = max(self._surf_h, s_h)

        if not self._pygame_init:
            self._pygame_init = True
            self.init_pygame()
        else:
            self._game_frame.config(width=self._surf_w, height=self._surf_h)
            self._game_frame.grid(row=0, column=1)

        self._display = pygame.display.set_mode((self._surf_w, self._surf_h))

        self._surfs.append(surf)
        self._surf_coords[surf] = (self._surf_w-s_w, 0)

        self._display.blits(list(self._surf_coords.items()))

    def refresh(self):
        for fig in list(self._figs):
            self.refresh_figure(fig)
            self._root.update()

        if not self._display is None:
            self._display.blits(list(self._surf_coords.items()))
            pygame.display.flip()
