#!/usr/bin/python
"""
    grbl-i2g G-Code Generator
    An heavily modified version of dmap2gcode by Scorch
    Suited for using with GRBL CNC controller, it uses PIL and Numpy by default
    and has no code to integrate into linuxcnc.   

    Copyright (C) <2015> Onekk
    Source was used from the following works:
        image-to-gcode.py   2005 Chris Radek chris@timeguy.com
        image-to-gcode.py   2006 Jeff Epler
        Author.py(linuxcnc) 2007 Jeff Epler  jepler@unpythonic.net
        dmap2gcode.py       2015 Scorch
        
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Version 0.01
    - Initial code based on version 0.06 by scorch
    - Removed all dmap2gcode names in the program to avoid confusion with the
      original version.
    - Removed the options for not using PIL and Numpy
    - Removed all AXIS reference (we use it on GRBL not in LinuxCNC)
    - Added a "Save Configuration" menu item 

"""
version = '0.01'
p_name = "grbl-i2g"


import sys
VERSION = sys.version_info[0]

if sys.platform.startswith('win32'):
    config_file = "." + p_name + "rc"
elif sys.platform.startswith('linux'):
    config_file = p_name + ".ngc"
            
if VERSION == 3:
    from tkinter import *
    from tkinter.filedialog import *
    import tkinter.messagebox
    MAXINT = sys.maxsize
    
else:
    from Tkinter import *
    from tkFileDialog import *
    #import ttk
    import tkMessageBox
    MAXINT = sys.maxint

if VERSION < 3 and sys.version_info[1] < 6:
    def next(item):
        return item.next()

from math import *
from time import time
import os
import re
import binascii
import getopt
import operator

from PIL import Image
from PIL import ImageTk
from PIL import ImageOps
try:
  # this is valid for distro derivated from Debian (Debian Ubuntu, Linux Mint) 
  from PIL.Image import core as _imaging 
except:
    # for other distro
    import _imaging 

try:
    import numpy.numarray as numarray
    import numpy.core
    olderr = numpy.core.seterr(divide='ignore')
    plus_inf = (numarray.array((1.,))/0.)[0]
    numpy.core.seterr(**olderr)
except ImportError:
    import numarray, numarray.ieeespecial
    plus_inf = numarray.ieeespecial.inf

#Setting QUIET to True will stop almost all console messages
QUIET = False
STOP_CALC = 0

class Application(Frame):
    def __init__(self, master):
        Frame.__init__(self, master)
        self.w = 960
        self.h = 490
        self.canvas_width = self.w-730
        frame = Frame(master, width= self.w, height=self.h)
        self.master = master
        self.x = -1
        self.y = -1
                
        self.createWidgets()

    def createWidgets(self):
        self.initComplete = 0
        self.master.bind("<Configure>", self.Master_Configure)
        self.master.bind('<Enter>', self.bindConfigure)
        self.master.bind('<Escape>', self.KEY_ESC)
        self.master.bind('<F1>', self.KEY_F1)
        self.master.bind('<F2>', self.KEY_F2)
        self.master.bind('<F3>', self.KEY_F3)
        self.master.bind('<F4>', self.KEY_F4)
        self.master.bind('<F5>', self.KEY_F5) #self.Recalculate_Click)
        self.master.bind('<Control-g>', self.KEY_CTRL_G)

        self.show_axis = BooleanVar()
        self.invert = BooleanVar()
        self.normalize = BooleanVar()
        self.cuttop = BooleanVar()
        self.cutperim = BooleanVar()
        self.no_arcs = BooleanVar()
        self.grbl_flag = BooleanVar()
        
        self.origin = StringVar()
        self.yscale = StringVar()
        self.Xscale = StringVar()
        self.pixsize = StringVar()
        self.toptol = StringVar()
        self.tool = StringVar()
        self.dia  = StringVar()
        self.v_angle = StringVar()
        self.scanpat = StringVar()
        self.scandir = StringVar()
        self.f_feed = StringVar()
        self.p_feed = StringVar()
        self.stepover = StringVar()
        self.z_cut = StringVar()
        self.z_safe = StringVar()
        self.rough_tool = StringVar()
        self.rough_dia = StringVar()
        self.rough_v_angle = StringVar()
        self.rough_scanpat = StringVar()
        self.rough_scandir = StringVar()
        self.rough_r_feed = StringVar()
        self.rough_p_feed = StringVar()
        self.rough_stepover = StringVar()
        self.rough_depth_pp = StringVar()
        self.rough_offset = StringVar()
        self.units = StringVar()
        self.plungetype = StringVar()
        self.lace_bound = StringVar()
        self.cangle = StringVar()
        self.tolerance = StringVar()
        self.splitstep = StringVar()
        self.funits = StringVar()
        self.gpre = StringVar()
        self.gpost = StringVar()
        self.maxcut = StringVar()
        self.current_input_file = StringVar()

        #####################################################
        #               INITIALIZE VARIABLES                #
        #  if you want to change a default setting          #
        #  this is the place to do it                       #
        #####################################################
        
        self.show_axis.set(1)
        self.invert.set(0)
        self.normalize.set(0)
        self.cuttop.set(1)
        self.cutperim.set(1)
        self.no_arcs.set(0)
        self.grbl_flag.set(1)

        self.yscale.set("5.0")
        self.Xscale.set("0")
        self.pixsize.set("0")
        self.toptol.set("-0.005")

        self.tool.set("Flat")           # Options are "Ball", "Flat", "V"
        self.v_angle.set("45")
        self.f_feed.set("1500")
        self.p_feed.set("250")
        self.stepover.set("0.04")
        self.z_cut.set("-10.00")
        self.z_safe.set("5")       
        self.dia.set("0.80")
        self.scanpat.set("Rows")
        self.scandir.set("Positive")    # Options are "Alternating", 
                                        #     "Positive"   , "Negative",
                                        #     "Up Mill", "Down Mill"
                                        
        self.rough_tool.set("Flat")     # Options are "Ball", "Flat", "V"                                
        self.rough_v_angle.set("45")
        self.rough_r_feed.set("1500.00")
        self.rough_p_feed.set("250.00")
        self.rough_stepover.set("0.04")
        self.rough_depth_pp.set("0.10")
        self.rough_offset.set("0.02")
        self.rough_dia.set("3.00")
        self.rough_scanpat.set("Rows")
        self.rough_scandir.set("Positive")  # Options are "Alternating", 
                                            #   "Positive"   , "Negative",
                                            #   "Up Mill", "Down Mill"        

        self.origin.set("Default")      # Options are "Default",
                                        # "Top-Left", "Top-Center", "Top-Right",
                                        # "Mid-Left", "Mid-Center", "Mid-Right",
                                        # "Bot-Left", "Bot-Center", "Bot-Right"

        self.units.set("mm")            # Options are "in" and "mm"
        self.plungetype.set("simple")

        self.lace_bound.set("None")     # Options "Full", "None", "??"
        self.cangle.set("45.0")
        self.tolerance.set("0.001")
        self.splitstep.set("0")        # Options 


        self.z1_cut = "-100.00"

        self.NGC_FILE     = (os.path.expanduser("~")+"/None")
        self.IMAGE_FILE   = (os.path.expanduser("~")+"/None")
        
        self.aspect_ratio =  0
        self.SCALE = 1
        
        self.gcode = []
        self.segID = []

        # PAN and ZOOM STUFF
        self.panx = 0
        self.panx = 0
        self.lastx = 0
        self.lasty = 0
        
        # Derived variables
        if self.units.get() == 'in':
            self.funits.set('in/min')
        else:
            self.units.set('mm')
            self.funits.set('mm/min')

        self.ui_TKimage = PhotoImage(format='gif',data=
         'R0lGODdhggDpAIAAAAAAAP///ywAAAAAggDpAAAC/oyPqcvtD6OctNqLs968'
        +'+w+G4kiW5ommCMC27uuqsgjXNjzn1833rQ5s+IbEoJGILBplyaZyWXJKn9DP'
        +'9DqserDcrDbTDfu+FrGZR56c17b0gw2vuRnx+muesOtj+LowPreGceYWtiVG'
        +'1jWiqMVlwriEpeJ4dJUjGWRJcVOhqTMl4RQK+ikKQfkmdWnqYHaqOslKx5ba'
        +'xGT7B9gqG4U7a1eLlMK7sseSm3RCfGDMt7AM4qvQfPcr3Hs9TV39nIztpb3N'
        +'nScd7R0u7kyeHXK+nq5ezG5OxQwv1z3f4S5/P26vjwM/A57ywUI3pl3Ag8HK'
        +'ERy4ASI0ZAEDQNSw0GEE/oYAwe3LWHGjRo2DQHq0iM/ayY4JP9Z7SKUHxZb9'
        +'aEY0KZNlG4MrUb4Eo+9cxYshywS9NlLiz5I9vS1TutToy2SkauZ8ZxPoT2FV'
        +'rXJCePXmVj080YA1KxCnLq9f2e50mdUnMKxhdbZN23TP2bt23+LNO9dtSsH/'
        +'MKqlRRct4XiGxwbu6xcyjkNLH0suLDeuVsBwyvKF2XOzZj+JP2euK7Yy4tKR'
        +'QWsWjdq1Ic+mL8KOfRrVYsxFmUZ1Rbu1bNN/Q+eGRrK376jHfxNl3rgoR9bE'
        +'lS+Hbrs5ce2YKWeX/Iok9x/fvkfv/drKdLjmcdObeB065Fhdb1vvWKqg1PV4'
        +'/sYbV8lff/5Jh4iAAPqTnoEI3tffggkaeBmCEKrh4GQTjlLhMRdukuGGOyzo'
        +'YXzNhJjaiCQWR9qJ7K2monqC3ALAhJAMI1whAdLgXh/yoagYhAx+KN5+Grr4'
        +'IJA3UrhdJztyiMl5SWJYpEgF2vfkLkvONKOQP0bIGJSEUPmjeXtNaWSTDeVI'
        +'HXAi6jfmYFh2VuJswbmZ5otxqrnbkHXaeeeXXOo5oJ/erZWNOOVluI0yiCYK'
        +'46IpruKooJFEqtsXlMJn6aVo2qhplx522mKb8IR6ZjqkemniqUrqpSqYlbbq'
        +'KqawlinrrE7+Z+t7m+Y6aI+8kiDmr7dWKeywNRa7/qKvyPaq7LI8EuusltBG'
        +'y2SU1CJp7bVSZqttqcd2uyq34B747bioTmsuueWmayWu7L657rtzWigvtuLK'
        +'G2y9ecaob7W79vsnvwBv++/AW+p7cL0J43slwPn2+zDCC7MbscITU9wwxBlL'
        +'7O7Aw8Xr8cXpimwuyeOaDG7F7wYZMssOH8kxzCu/qjHNFstZM5kMS4oxiyM/'
        +'ejJZJbOacqrXGnq0JrXC2qTMrernNKlsBuop0/UFXLWqU2NNXq5b75u1il/v'
        +'Se+pY5Pd3YlniwqyglcTvHEaa6vb7IVzw1u321HjjW4jb9PacaZ7e1swFHfb'
        +'GzekS4eLMrB/P1u44otLV3svjYNTHrnlk8dKp+GXr5l3fi7rmvgimydbuemN'
        +'Mx645qlD3vcjpTMb+gyzE/k6jrXT1zrvZXue+6E6Zv5y7C137jHcyS/PfPPO'
        +'Pw999NJPT3311gdRAAA7')
        self.im  = self.ui_TKimage
        self.wim = self.ui_TKimage.width()
        self.him = self.ui_TKimage.height()
        self.aspect_ratio =  float(self.wim-1) / float(self.him-1)

        ########################################################################
        #                         G-Code Default Preamble                      #
        ########################################################################
        # G17        ; sets XY plane                                           #
        # G90        ; Fixed cycle                                             #
        # M3 S3000   ; Spindle start at 3000                                   #
        # M7         ; Turn mist coolant on                                    #
        ########################################################################
        self.gpre.set("G17 G90 M3 S3000 M7")

        ########################################################################
        #                        G-Code Default Postamble                      #
        ########################################################################
        # M5 ; Stop Spindle                                                    #
        # M9 ; Turn all coolant off                                            #
        # M2 ; End Program                                                     #
        ########################################################################
        self.gpost.set("M5 M9 M2")
        
        self.statusMessage = StringVar()
        self.statusMessage.set("Welcome to " + p_name)
        
        ##########################################################################
        ###                     END INITILIZING VARIABLES                      ###
        ##########################################################################

        # make a Status Bar
        self.statusbar = Label(self.master, textvariable=self.statusMessage, \
                                   bd=1, relief=SUNKEN , height=1)
        self.statusbar.pack(anchor=SW, fill=X, side=BOTTOM)
        

        # Buttons
        self.Save_Button = Button(self.master,text="Save G-Code",command=self.menu_File_Save_G_Code_File_Finish)
        # Canvas
        lbframe = Frame(self.master)
        self.PreviewCanvas_frame = lbframe
        self.PreviewCanvas = Canvas(lbframe, width=self.canvas_width, \
                                        height=self.h-200, background="grey")
        self.PreviewCanvas.pack(side=LEFT, fill=BOTH, expand=1)
        self.PreviewCanvas_frame.place(x=230, y=10)

        self.PreviewCanvas.bind("<1>"        , self.mousePanStart)
        self.PreviewCanvas.bind("<B1-Motion>", self.mousePan)
        self.PreviewCanvas.bind("<2>"        , self.mousePanStart)
        self.PreviewCanvas.bind("<B2-Motion>", self.mousePan)
        self.PreviewCanvas.bind("<3>"        , self.mousePanStart)
        self.PreviewCanvas.bind("<B3-Motion>", self.mousePan)

        # Left Column #
        self.Label_font_prop = Label(self.master,text="Image Size:", anchor=W)

        self.Label_Yscale = Label(self.master,text="Image Height", anchor=CENTER)
        self.Label_Yscale_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_Yscale = Entry(self.master,width="15")
        self.Entry_Yscale.configure(textvariable=self.yscale)
        self.Entry_Yscale.bind('<Return>', self.Recalculate_Click)
        self.yscale.trace_variable("w", self.Entry_Yscale_Callback)
        self.NormalColor =  self.Entry_Yscale.cget('bg')

        self.Label_Yscale2 = Label(self.master,text="Image Width", anchor=CENTER)
        self.Label_Yscale2_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Label_Yscale2_val = Label(self.master,textvariable=self.Xscale, anchor=W)

        self.Label_PixSize = Label(self.master,text="Pixel Size", anchor=CENTER)
        self.Label_PixSize_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Label_PixSize_val = Label(self.master,textvariable=self.pixsize, anchor=W)
        
        self.Label_pos_orient = Label(self.master,text="Image Properties:",\
                                          anchor=W)

        self.Label_Origin      = Label(self.master,text="Origin", anchor=CENTER )
        self.Origin_OptionMenu = OptionMenu(root, self.origin,
                                            "Top-Left",
                                            "Top-Center",
                                            "Top-Right",
                                            "Mid-Left",
                                            "Mid-Center",
                                            "Mid-Right",
                                            "Bot-Left",
                                            "Bot-Center",
                                            "Bot-Right",
                                            "Default", command=self.Recalculate_RQD_Click)
        
        #Radio Button
        self.Label_Invert_Color_FALSE = Label(self.master,text="Depth Color")
        self.Radio_Invert_Color_FALSE = Radiobutton(self.master,text="Black", value=False,
                                         width="100", anchor=W)
        self.Radio_Invert_Color_FALSE.configure(variable=self.invert )
        #self.Label_Invert_Color_TRUE = Label(self.master,text=" ")
        self.Radio_Invert_Color_TRUE = Radiobutton(self.master,text="White", value=True,
                                         width="100", anchor=W)
        self.Radio_Invert_Color_TRUE.configure(variable=self.invert )


        self.Label_normalize = Label(self.master,text="Normalize Depth")
        self.CB_normalize = Checkbutton(self.master,text=" ", anchor=W)
        self.CB_normalize.configure(variable=self.normalize)

        self.separator1 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.separator2 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.separator3 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.separator4 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.ROUGH_sep1 = Frame(self.master, height=self.w-30, bd=1, relief=SUNKEN)
        self.ROUGH_sep2 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.ROUGH_sep3 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
        self.ROUGH_sep4 = Frame(self.master, height=2, bd=1, relief=SUNKEN)
                
        self.Label_CutTop = Label(self.master,text="Cut Top Surface")
        self.CB_CutTop = Checkbutton(self.master,text=" ", \
                                              anchor=W, command=self.Set_Input_States)
        self.CB_CutTop.configure(variable=self.cuttop)

        self.Label_Toptol = Label(self.master,text="Top Tolerance", anchor=CENTER )
        self.Label_Toptol_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_Toptol = Entry(self.master,width="15")
        self.Entry_Toptol.configure(textvariable=self.toptol)
        self.toptol.trace_variable("w", self.Entry_Toptol_Callback)
 
        # End Left Column #

        # Right Column #
        self.Label_tool_opt = Label(self.master,text="Finish Tool Properties:", anchor=W)

        self.Label_ToolDIA = Label(self.master,text="Tool DIA")
        self.Label_ToolDIA_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_ToolDIA = Entry(self.master,width="15")
        self.Entry_ToolDIA.configure(textvariable=self.dia)
        self.Entry_ToolDIA.bind('<Return>', self.Recalculate_Click)
        self.dia.trace_variable("w", self.Entry_ToolDIA_Callback)

        self.Label_Tool      = Label(self.master,text="Tool End", anchor=CENTER )
        self.Tool_OptionMenu = OptionMenu(root, self.tool, "Ball","V","Flat",\
                                               command=self.Set_Input_States_Event)
                
        self.Label_Vangle = Label(self.master,text="V-Bit Angle", anchor=CENTER )
        self.Entry_Vangle = Entry(self.master,width="15")
        self.Entry_Vangle.configure(textvariable=self.v_angle)
        self.Entry_Vangle.bind('<Return>', self.Recalculate_Click)
        self.v_angle.trace_variable("w", self.Entry_Vangle_Callback)

        self.Label_gcode_opt = Label(self.master,text="Finish Gcode Properties:", anchor=W)

        self.Label_Scanpat      = Label(self.master,text="Scan Pattern", anchor=CENTER )
        self.ScanPat_OptionMenu = OptionMenu(root, self.scanpat, "Rows","Columns",
                                            "R then C", "C then R")

        self.Label_CutPerim = Label(self.master,text="Cut Perimeter")
        self.CB_CutPerim = Checkbutton(self.master,text=" ",
                                              anchor=W, command=self.Set_Input_States)
        self.CB_CutPerim.configure(variable=self.cutperim)

        self.Label_Scandir      = Label(self.master,text="Scan Direction", anchor=CENTER )
        self.ScanDir_OptionMenu = OptionMenu(root, self.scandir, "Alternating", "Positive",
                                            "Negative", "Up Mill", "Down Mill")
        
        self.Label_Feed = Label(self.master,text="Feed Rate")
        self.Label_Feed_u = Label(self.master,textvariable=self.funits, anchor=W)
        self.Entry_Feed = Entry(self.master,width="15")
        self.Entry_Feed.configure(textvariable=self.f_feed)
        self.Entry_Feed.bind('<Return>', self.Recalculate_Click)
        self.f_feed.trace_variable("w", self.Entry_Feed_Callback)

        self.Label_p_feed = Label(self.master,text="Plunge Feed", anchor=CENTER )
        self.Label_p_feed_u = Label(self.master,textvariable=self.funits, anchor=W)
        self.Entry_p_feed = Entry(self.master,width="15")
        self.Entry_p_feed.configure(textvariable=self.p_feed)
        self.Entry_p_feed.bind('<Return>', self.Recalculate_Click)
        self.p_feed.trace_variable("w", self.Entry_p_feed_Callback)

        self.Label_StepOver = Label(self.master,text="Stepover", anchor=CENTER )
        self.Label_StepOver_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_StepOver = Entry(self.master,width="15")
        self.Entry_StepOver.configure(textvariable=self.stepover)
        self.Entry_StepOver.bind('<Return>', self.Recalculate_Click)
        self.stepover.trace_variable("w", self.Entry_StepOver_Callback)

        self.Label_Zsafe = Label(self.master,text="Z Safe")
        self.Label_Zsafe_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_Zsafe = Entry(self.master,width="15")
        self.Entry_Zsafe.configure(textvariable=self.z_safe)
        self.Entry_Zsafe.bind('<Return>', self.Recalculate_Click)
        self.z_safe.trace_variable("w", self.Entry_Zsafe_Callback)

        self.Label_Zcut = Label(self.master,text="Max Cut Depth")
        self.Label_Zcut_u = Label(self.master,textvariable=self.units, anchor=W)
        self.Entry_Zcut = Entry(self.master,width="15")
        self.Entry_Zcut.configure(textvariable=self.z_cut)
        self.Entry_Zcut.bind('<Return>', self.Recalculate_Click)
        self.z_cut.trace_variable("w", self.Entry_Zcut_Callback)      
        # End Right Column #
        
        # Third column
        self.ROUGH_Label_tool_opt = Label(self.master,text="Roughing Tool Properties:", anchor=W)
        self.ROUGH_Label_Tool = Label(self.master,text="R. Tool End", anchor=CENTER )
        self.rough_tool_OptionMenu = OptionMenu(self.master, self.rough_tool,
                                                "Ball","V","Flat",
                                               command=self.Set_Input_States_Event_ROUGH)        
        
        
        self.ROUGH_Label_ToolDIA = Label(self.master,text="R. Tool DIA")
        self.ROUGH_Label_ToolDIA_u = Label(self.master,textvariable=self.units, anchor=W)
        self.ROUGH_Entry_ToolDIA = Entry(self.master,width="15")
        self.ROUGH_Entry_ToolDIA.configure(textvariable=self.rough_dia)
        self.ROUGH_Entry_ToolDIA.bind('<Return>', self.Recalculate_Click)
        self.rough_dia.trace_variable("w", self.ROUGH_Entry_ToolDIA_Callback)
        
        self.ROUGH_Label_Vangle = Label(self.master,text="R. V-Bit Angle", anchor=CENTER )
        self.ROUGH_Entry_Vangle = Entry(self.master,width="15")
        self.ROUGH_Entry_Vangle.configure(textvariable=self.rough_v_angle)
        self.ROUGH_Entry_Vangle.bind('<Return>', self.Recalculate_Click)
        self.rough_v_angle.trace_variable("w", self.ROUGH_Entry_Vangle_Callback)

        self.ROUGH_Label_gcode_opt = Label(self.master,text="Roughing Gcode Properties:", anchor=W)

        self.ROUGH_Label_Scanpat = Label(self.master,text="R. Scan Pattern", anchor=CENTER )
        self.ROUGH_ScanPat_OptionMenu = OptionMenu(self.master, self.rough_scanpat, "Rows","Columns",\
                                           "R then C", "C then R")

        self.ROUGH_Label_Scandir = Label(self.master,text="R. Scan Dir.", anchor=CENTER )
        self.ROUGH_ScanDir_OptionMenu = OptionMenu(self.master, self.rough_scandir, "Alternating", "Positive",
                                            "Negative", "Up Mill", "Down Mill")

        self.ROUGH_Label_Feed = Label(self.master,text="R. Feed Rate")
        self.ROUGH_Label_Feed_u = Label(self.master,textvariable=self.funits, anchor=W)
        self.ROUGH_Entry_Feed = Entry(self.master,width="15")
        self.ROUGH_Entry_Feed.configure(textvariable=self.rough_r_feed)
        self.ROUGH_Entry_Feed.bind('<Return>', self.Recalculate_Click)
        self.rough_r_feed.trace_variable("w", self.ROUGH_Entry_Feed_Callback)

        self.ROUGH_Label_p_feed = Label(self.master,text="R. Plunge Feed", anchor=CENTER )
        self.ROUGH_Label_p_feed_u = Label(self.master,textvariable=self.funits, anchor=W)
        self.ROUGH_Entry_p_feed = Entry(self.master,width="15")
        self.ROUGH_Entry_p_feed.configure(textvariable=self.rough_p_feed)
        self.ROUGH_Entry_p_feed.bind('<Return>', self.Recalculate_Click)
        self.rough_p_feed.trace_variable("w", self.Entry_p_feed_Callback)

        self.ROUGH_Label_StepOver = Label(self.master,text="R. Stepover", anchor=CENTER )
        self.ROUGH_Label_StepOver_u = Label(self.master,textvariable=self.units, anchor=W)
        self.ROUGH_Entry_StepOver = Entry(self.master,width="15")
        self.ROUGH_Entry_StepOver.configure(textvariable=self.rough_stepover)
        self.ROUGH_Entry_StepOver.bind('<Return>', self.Recalculate_Click)
        self.rough_stepover.trace_variable("w", self.ROUGH_Entry_StepOver_Callback)

        self.ROUGH_Label_roughing_props = Label(self.master,text="Roughing Properties:",anchor=W)    

        self.ROUGH_Label_Roffset = Label(self.master,text="R. Offset", anchor=CENTER )
        self.ROUGH_Label_Roffset_u = Label(self.master,textvariable=self.units, anchor=W)
        self.ROUGH_Entry_Roffset = Entry(self.master,width="15")
        self.ROUGH_Entry_Roffset.configure(textvariable=self.rough_offset)
        self.ROUGH_Entry_Roffset.bind('<Return>', self.Recalculate_Click)
        self.rough_offset.trace_variable("w", self.ROUGH_Entry_Roffset_Callback)

        self.ROUGH_Label_Rdepth = Label(self.master,text="R. Depth/Pass", anchor=CENTER )
        self.ROUGH_Label_Rdepth_u = Label(self.master,textvariable=self.units, anchor=W)
        self.ROUGH_Entry_Rdepth = Entry(self.master,width="15")
        self.ROUGH_Entry_Rdepth.configure(textvariable=self.rough_depth_pp)
        self.ROUGH_Entry_Rdepth.bind('<Return>', self.Recalculate_Click)
        self.rough_depth_pp.trace_variable("w", self.ROUGH_Entry_Rdepth_Callback)

        self.ROUGH_Save = Button(self.master,text="Save\nRoughing G-Code",\
                                 command=self.menu_File_Save_G_Code_File_Rough)

        #GEN Setting Window Entry initializations
        self.Entry_Sspeed=Entry()
        self.Entry_BoxGap = Entry()
        self.Entry_ContAngle = Entry()
        self.Entry_Tolerance = Entry()

        # Make Menu Bar
        self.menuBar = Menu(self.master, relief = "raised", bd=2)

        top_File = Menu(self.menuBar, tearoff=0)
        top_File.add("command", label = "Open G-Code File", \
                         command = self.menu_File_Open_G_Code_File)

        top_File.add("command", label = "Open Image File", \
                             command = self.menu_File_Open_IMAGE_File)

        top_File.add("command", label = "Save G-Code File", \
                         command = self.menu_File_Save_G_Code_File_Finish)
        top_File.add("command", label = "Save Roughing G-Code File", \
                         command = self.menu_File_Save_G_Code_File_Rough)
        top_File.add("command", label = "Exit", command = self.menu_File_Quit)
        self.menuBar.add("cascade", label="File", menu=top_File)

        top_Edit = Menu(self.menuBar, tearoff=0)
        top_Edit.add("command", label = "Copy G-Code Data to Clipboard", \
                         command = self.CopyClipboard_GCode)
        self.menuBar.add("cascade", label="Edit", menu=top_Edit)

        top_View = Menu(self.menuBar, tearoff=0)
        top_View.add("command", label = "Refresh", command = self.menu_View_Refresh)

        top_View.add_separator()

        top_View.add_checkbutton(label = "Show Origin Axis",  variable=self.show_axis , \
                                     command= self.menu_View_Refresh)

        self.menuBar.add("cascade", label="View", menu=top_View)

        top_Settings = Menu(self.menuBar, tearoff=0)
        top_Settings.add("command", label = "General Settings", \
                             command = self.GEN_Settings_Window)
        top_Settings.add("command", label = "Save Settings", \
                             command = self.Write_Settings)
        
        self.menuBar.add("cascade", label="Settings", menu=top_Settings)

        top_Help = Menu(self.menuBar, tearoff=0)
        top_Help.add("command", label = "About", command = self.menu_Help_About)
        self.menuBar.add("cascade", label="Help", menu=top_Help)

        self.master.config(menu=self.menuBar)

        #####################################################
        #        Config File and command line options       #
        #####################################################

        home_config = os.path.expanduser("~") + "/" + config_file 
        print home_config
        
        if ( os.path.isfile(config_file) ):
            self.Open_G_Code_File(config_file)
            print "loaded config file"
        elif ( os.path.isfile(home_config) ):
            self.Open_G_Code_File(home_config)
            print "loaded home config"

        opts, args = None, None
        try:
            opts, args = getopt.getopt(sys.argv[1:], "hg:",["help", "gcode_file"])
        except:
            fmessage('Unable interpret command line options')
            sys.exit()
        for option, value in opts:
            if option in ('-h','--help'):
                fmessage(' ')
                fmessage('Usage: python ' + p_name + '.py [-g file]')
                fmessage('-g    : ' + p_name + 'gcode output file to read (also --gcode_file)')
                fmessage('-h    : print this help (also --help)\n')
                sys.exit()
            if option in ('-g','--gcode_file'):
                    self.Open_G_Code_File(value)


    def entry_set(self, val2, calc_flag=0, new=0):
        if calc_flag == 0 and new==0:
            try:
                self.statusbar.configure( bg = 'yellow' )
                val2.configure( bg = 'yellow' )
                self.statusMessage.set(" Recalculation required.")
            except:
                pass
        elif calc_flag == 3:
            try:
                val2.configure( bg = 'red' )
                self.statusbar.configure( bg = 'red' )
                self.statusMessage.set(" Value should be a number. ")
            except:
                pass
        elif calc_flag == 2:
            try:
                self.statusbar.configure( bg = 'red' )
                val2.configure( bg = 'red' )
            except:
                pass
        elif (calc_flag == 0 or calc_flag == 1) and new==1 :
            try:
                self.statusbar.configure( bg = 'white' )
                self.statusMessage.set(" ")
                val2.configure( bg = 'white' )
            except:
                pass
        elif (calc_flag == 1) and new==0 :
            try:
                self.statusbar.configure( bg = 'white' )
                self.statusMessage.set(" ")
                val2.configure( bg = 'white' )
            except:
                pass

        elif (calc_flag == 0 or calc_flag == 1) and new==2:
            return 0
        return 1

    def Write_Settings(self):
        config = []
        config.append('Config file for ' + p_name + ' ' + version + '.py')
        config.append('by Onekk - 2015')
        # BOOL
        config.append('cfg_set show_axis   %s' %( int(self.show_axis.get())))
        config.append('cfg_set invert      %s' %( int(self.invert.get())))
        config.append('cfg_set normalize   %s' %( int(self.normalize.get())))
        config.append('cfg_set cuttop      %s' %( int(self.cuttop.get())))
        config.append('cfg_set cutperim    %s' %( int(self.cutperim.get())))
        config.append('cfg_set no_arcs     %s' %( int(self.no_arcs.get())))
        config.append('cfg_set grbl_flag   %s' %( int(self.grbl_flag.get())))        
        # STRING.get()
        config.append('cfg_set yscale      %s'  %( self.yscale.get()))
        config.append('cfg_set toptol      %s'  %( self.toptol.get()))
        config.append('cfg_set vangle      %s'  %( self.v_angle.get()))
        config.append('cfg_set fh_stepover %s'  %( self.stepover.get()))
        config.append('cfg_set fh_r_feed   %s'  %( self.f_feed.get()))
        config.append('cfg_set fh_p_feed   %s'  %( self.p_feed.get()))
        config.append('cfg_set z_safe      %s'  %( self.z_safe.get()))
        config.append('cfg_set z_cut       %s'  %( self.z_cut.get()))
        config.append('cfg_set diatool     %s'  %( self.dia.get()))
        config.append('cfg_set origin      %s'  %( self.origin.get()))
        config.append('cfg_set tool        %s'  %( self.tool.get()))
        config.append('cfg_set units       %s'  %( self.units.get()))
        config.append('cfg_set plunge      %s'  %( self.plungetype.get()))

        config.append('cfg_set lace        %s'  %( self.lace_bound.get()))
        config.append('cfg_set cangle      %s'  %( self.cangle.get()))        
        config.append('cfg_set tolerance   %s'  %( self.tolerance.get())) 
        config.append('cfg_set splitstep   %s'  %( self.splitstep.get()))
        config.append('cfg_set gpre        \042%s\042' %( self.gpre.get()))
        config.append('cfg_set gpost       \042%s\042' %( self.gpost.get()))
        config.append('cfg_set fh_scanpat  \042%s\042' %( self.scanpat.get()))
        config.append('cfg_set fh_scandir  \042%s\042' %( self.scandir.get()))
        config.append('cfg_set imagefile   \042%s\042' %( self.IMAGE_FILE))
        config.append('cfg_set rough_tool  %s'  %( self.rough_tool.get()))
        config.append('cfg_set rough_dia   %s'  %( self.rough_dia.get()))
        config.append('cfg_set rough_v_angle  %s'  %( self.rough_v_angle.get()))
        config.append('cfg_set rough_r_feed   %s'  %( self.rough_r_feed.get()))
        config.append('cfg_set rough_p_feed   %s'  %( self.rough_p_feed.get()))
        config.append('cfg_set rough_stepover %s'  %( self.rough_stepover.get()))
        config.append('cfg_set rough_depth_pp %s'  %( self.rough_depth_pp.get()))
        config.append('cfg_set rough_offset   %s'  %( self.rough_offset.get()))
        config.append('cfg_set rough_scanpat  \042%s\042' %( self.rough_scanpat.get()))
        config.append('cfg_set rough_scandir  \042%s\042' %( self.rough_scandir.get()))
        
        home_config = os.path.expanduser("~") + "/" + config_file         
        
        with open(home_config, 'wb') as confile:
            for line in config:
                confile.write(line + "\n")


    def WriteGCode(self,rough_flag = 0):
        global Zero
        header = []
        header.append('(Code generated by ' + p_name + ' ' + version + '.py)')
        header.append('(by Onekk - 2015 )')
        header.append('(Settings used to generate this file)')
        header.append('(=========================================)')
        # BOOL
        header.append('(cfg_set show_axis   %s )' %( int(self.show_axis.get())))
        header.append('(cfg_set invert      %s )' %( int(self.invert.get())))
        header.append('(cfg_set normalize   %s )' %( int(self.normalize.get())))
        header.append('(cfg_set cuttop      %s )' %( int(self.cuttop.get())))
        header.append('(cfg_set cutperim    %s )' %( int(self.cutperim.get())))
        header.append('(cfg_set no_arcs     %s )' %( int(self.no_arcs.get())))
        header.append('(cfg_set grbl_flag   %s )' %( int(self.grbl_flag.get())))
        # STRING.get()
        header.append('(cfg_set yscale      %s )'  %( self.yscale.get()))
        header.append('(cfg_set toptol      %s )'  %( self.toptol.get()))
        header.append('(cfg_set vangle      %s )'  %( self.v_angle.get()))
        header.append('(cfg_set fh_stepover %s )'  %( self.stepover.get()))
        header.append('(cfg_set fh_r_feed   %s )'  %( self.f_feed.get()))
        header.append('(cfg_set fh_p_feed   %s )'  %( self.p_feed.get()))
        header.append('(cfg_set z_safe      %s )'  %( self.z_safe.get()))
        header.append('(cfg_set z_cut       %s )'  %( self.z_cut.get()))
        header.append('(cfg_set diatool     %s )'  %( self.dia.get()))
        header.append('(cfg_set origin      %s )'  %( self.origin.get()))
        header.append('(cfg_set tool        %s )'  %( self.tool.get()))
        header.append('(cfg_set units       %s )'  %( self.units.get()))
        header.append('(cfg_set plunge      %s )'  %( self.plungetype.get()))

        header.append('(cfg_set lace        %s )'  %( self.lace_bound.get()))
        header.append('(cfg_set cangle      %s )'  %( self.cangle.get()))        
        header.append('(cfg_set tolerance   %s )'  %( self.tolerance.get())) 
        header.append('(cfg_set splitstep   %s )'  %( self.splitstep.get()))
        header.append('(cfg_set gpre        \042%s\042 )' %( self.gpre.get()))
        header.append('(cfg_set gpost       \042%s\042 )' %( self.gpost.get()))
        header.append('(cfg_set fh_scanpat  \042%s\042 )' %( self.scanpat.get()))
        header.append('(cfg_set fh_scandir  \042%s\042 )' %( self.scandir.get()))
        header.append('(cfg_set imagefile   \042%s\042 )' %( self.IMAGE_FILE))
        header.append('(cfg_set rough_tool   %s )'  %( self.rough_tool.get()))
        header.append('(cfg_set rough_dia    %s )'  %( self.rough_dia.get()))
        header.append('(cfg_set rough_v_angle  %s )'  %( self.rough_v_angle.get()))
        header.append('(cfg_set rough_r_feed   %s )'  %( self.rough_r_feed.get()))
        header.append('(cfg_set rough_p_feed   %s )'  %( self.rough_p_feed.get()))
        header.append('(cfg_set rough_stepover %s )'  %( self.rough_stepover.get()))
        header.append('(cfg_set rough_depth_pp %s )'  %( self.rough_depth_pp.get()))
        header.append('(cfg_set rough_offset   %s )'  %( self.rough_offset.get()))
        header.append('(cfg_set rough_scanpat  \042%s\042 )' %( self.rough_scanpat.get()))
        header.append('(cfg_set rough_scandir  \042%s\042 )' %( self.rough_scandir.get()))
        header.append('(==========================================)')

        pil_format = False
        
        try:
            test = self.im.width()
        except:
            try:
                test = self.im.size
                pil_format = True
            except:
                self.statusMessage.set("No Image Loaded")
                self.statusbar.configure(bg = 'red')
                return
        
        MAT = Image_Matrix()
        MAT.FromImage(self.im,pil_format)

        image_h       =  float(self.yscale.get())
        pixel_size    =  image_h / ( float(MAT.width) - 1.0)
        image_w       =  pixel_size * ( float(MAT.height) - 1.0)
        tolerance     =  float(self.tolerance.get())
        safe_z        =  float(self.z_safe.get())
        splitstep     =  float(self.splitstep.get())
        toptol        =  float(self.toptol.get())
        depth         = -float(self.z_cut.get())
        Cont_Angle    =  float(self.cangle.get())
        cutperim      =  int(self.cutperim.get())
        
        r_feed = 5000.00
            
        if rough_flag == 0:
            ######################################################
            tool_type     =  self.tool.get()
            
            tool_diameter =  float(self.dia.get())
            rough_depth   =  0.0 
            rough_offset  =  0.0 
            c_feed        =  float(self.f_feed.get())
            p_feed        =  float(self.p_feed.get())
            step          =  max(1, int(floor( float(self.stepover.get()) / pixel_size)))

            edge_offset   = 0
            ######################################################
            if self.tool.get() == "Flat":
                TOOL = make_tool_shape(endmill, tool_diameter, pixel_size)
            elif self.tool.get() == "V":
                v_angle = float(self.v_angle.get())
                TOOL = make_tool_shape(vee_common(v_angle), tool_diameter, pixel_size)
            else: #"Ball"
                TOOL = make_tool_shape(ball_tool, tool_diameter, pixel_size)
            ######################################################
                
            rows = 0
            columns = 0
            columns_first = 0
            if self.scanpat.get() != "Columns":
                rows = 1
            if self.scanpat.get() != "Rows":
                columns = 1 
            if self.scanpat.get() == "C then R":
                columns_first = 1

            ######################################################
            converter = self.scandir.get()
            lace_bound_val = self.lace_bound.get()
            ### END FINISH CUT STUFF ###
        else:
            ######################################################
            tool_type     =  self.rough_tool.get()
            
            rough_depth   =  float(self.rough_depth_pp.get())
            rough_offset  =  float(self.rough_offset.get())
            tool_diameter =  float(self.rough_dia.get())
            finish_dia    =  float(self.dia.get())
            c_feed     =  float(self.rough_r_feed.get())
            p_feed     =  float(self.rough_p_feed.get())
            step          =  max(1, int(floor( float(self.rough_stepover.get()) / pixel_size)))
            edge_offset = max(0, (tool_diameter - finish_dia)/2.0)
            ######################################################
            if self.rough_tool.get() == "Flat":
                TOOL = make_tool_shape(endmill, tool_diameter, pixel_size, rough_offset)
            elif self.tool.get() == "V":
                v_angle = float(self.rough_v_angle.get())
                TOOL = make_tool_shape(vee_common(v_angle), tool_diameter, pixel_size, rough_offset)
            else: #"Ball"
                TOOL = make_tool_shape(ball_tool, tool_diameter, pixel_size, rough_offset)

            rows = 0
            columns = 0
            columns_first = 0
            if self.rough_scanpat.get() != "Columns":
                rows = 1
            if self.rough_scanpat.get() != "Rows":
                columns = 1 
            if self.rough_scanpat.get() == "C then R":
                columns_first = 1

            converter = self.rough_scandir.get()
            lace_bound_val = self.lace_bound.get()
            
        ### END ROUGHING STUFF ###
            
        if converter == "Positive":
            conv_index = 0
            #fmessage("Positive")
            
        elif converter == "Negative":
            conv_index = 1
            #fmessage("Negative")
            
        elif converter == "Alternating":
            conv_index = 2
            #fmessage("Alternating")
            
        elif converter == "Up Mill":
            conv_index = 3
            #fmessage("Up Milling")
            
        elif converter == "Down Mill":
            conv_index = 4
            #fmessage("Down Mill")
        else:
            conv_index = 2
            fmessage("Converter Error: Setting to, Alternating")
        
        if rows: convert_rows = convert_makers[conv_index]()
        else: convert_rows = None
        if columns: convert_cols = convert_makers[conv_index]()
        else: convert_cols = None

        if lace_bound_val != "None" and rows and columns:
            
            slope = tan(Cont_Angle*pi/180)
            
            if columns_first:
                convert_rows = Reduce_Scan_Lace(convert_rows, slope, step+1)
            else:
                convert_cols = Reduce_Scan_Lace(convert_cols, slope, step+1)
            if lace_bound_val == "Full":
                if columns_first:
                    convert_cols = Reduce_Scan_Lace(convert_cols, slope, step+1)
                else:
                    convert_rows = Reduce_Scan_Lace(convert_rows, slope, step+1)

        if self.units.get() == "in":
            units = 'G20'
        else:
            units = 'G21'

        if self.cuttop.get() != True:
            if rows == 1:
                convert_rows = Reduce_Scan_Lace_new(convert_rows, toptol, 1)
            if columns == 1:
                convert_cols = Reduce_Scan_Lace_new(convert_cols, toptol, 1)

        no_arcs = self.no_arcs.get()

        if self.plungetype.get() == "arc" and (not no_arcs):
            Entry_cut   = ArcEntryCut(r_feed, .125)
        else:
            Entry_cut   = SimpleEntryCut(r_feed)

        if self.normalize.get():
            pass
            a = MAT.min()
            b = MAT.max()
            if a != b:
                MAT.minus(a)
                MAT.mult(1./(b-a))
        else:
            MAT.mult(1/255.0)
            
        xoffset = 0
        yoffset = 0            

        MAT.mult(depth)
        
        ##########################################
        #         ORIGIN LOCATING STUFF          #
        ##########################################

        minx = 0
        maxx = image_w
        miny = 0
        maxy = image_h
        midx = (minx + maxx)/2
        midy = (miny + maxy)/2

        CASE = str(self.origin.get())
        if     CASE == "Top-Left":
            x_zero = minx
            y_zero = maxy
        elif   CASE == "Top-Center":
            x_zero = midx
            y_zero = maxy
        elif   CASE == "Top-Right":
            x_zero = maxx
            y_zero = maxy
        elif   CASE == "Mid-Left":
            x_zero = minx
            y_zero = midy
        elif   CASE == "Mid-Center":
            x_zero = midx
            y_zero = midy
        elif   CASE == "Mid-Right":
            x_zero = maxx
            y_zero = midy
        elif   CASE == "Bot-Left":
            x_zero = minx
            y_zero = miny
        elif   CASE == "Bot-Center":
            x_zero = midx
            y_zero = miny
        elif   CASE == "Bot-Right":
            x_zero = maxx
            y_zero = miny
        elif   CASE == "Arc-Center":
            x_zero = 0
            y_zero = 0
        else:          #"Default"
            x_zero = 0
            y_zero = 0   

        xoffset = xoffset - x_zero
        yoffset = yoffset - y_zero
        
        if self.invert.get():
            MAT.mult(-1.0)
        else:
            MAT.minus(depth)
            
        self.gcode = []
        
        MAT.pad_w_zeros(TOOL)
        
        START_TIME=time()
        
        feed = [r_feed, c_feed, p_feed]
       
        self.gcode = generate(self, MAT, units, TOOL, pixel_size, step,
                             safe_z, tolerance, feed, 
                             convert_rows, convert_cols, columns_first,
                             cutperim, Entry_cut, rough_depth, rough_flag,
                             xoffset, yoffset, splitstep, header,
                             self.gpre.get(), self.gpost.get(), edge_offset,
                             no_arcs,self.grbl_flag)

    def CopyClipboard_GCode(self):
        self.clipboard_clear()
        if (self.Check_All_Variables() > 0):
            return
        self.WriteGCode()
        for line in self.gcode:
            self.clipboard_append(line+'\n')
        self.statusMessage.set("G-Code Sent to Clipboard")

    def CopyClipboard_SVG(self):
        self.clipboard_clear()
        self.WriteSVG()
        for line in self.svgcode:
            self.clipboard_append(line+'\n')

    def Quit_Click(self, event):
        self.statusMessage.set("Exiting!")
        root.destroy()

    def mousePanStart(self,event):
        self.panx = event.x
        self.pany = event.y

    def mousePan(self,event):
        all = self.PreviewCanvas.find_all()
        dx = event.x-self.panx
        dy = event.y-self.pany
        for i in all:
            self.PreviewCanvas.move(i, dx, dy)
        self.lastx = self.lastx + dx
        self.lasty = self.lasty + dy
        self.panx = event.x
        self.pany = event.y

    def Recalculate_Click(self, event):
        pass

    def Settings_ReLoad_Click(self, event):
        win_id=self.grab_current()

    def Close_Current_Window_Click(self):
        win_id=self.grab_current()
        win_id.destroy()

    def Stop_Click(self, event):
        global STOP_CALC
        STOP_CALC=1
        
    # Left Column #
    #############################
    def Entry_Yscale_Check(self):
        try:
            value = float(self.yscale.get())
            if  value <= 0.0:
                self.statusMessage.set(" Height should be greater than 0 ")
                return 2 # Value is invalid number
            else:
                self.Xscale.set("%.3f" %( self.aspect_ratio * float(self.yscale.get())) )
                self.pixsize.set("%.3f" %( float(self.yscale.get()) / (self.him - 1.0) ) )
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_Yscale_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Yscale, self.Entry_Yscale_Check(), new=1)        
    #############################
    def Entry_Toptol_Check(self):
        try:
            value = float(self.toptol.get())
            if  value > 0.0:
                self.statusMessage.set(" Tolerance should be less than or equal to 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_Toptol_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Toptol, self.Entry_Toptol_Check(), new=1)
    #############################
    # End Left Column #
    #############################
    
    #############################
    # Start Right Column #
    #############################
    def Entry_ToolDIA_Check(self):
        try:
            value = float(self.dia.get())
            if  value <= 0.0:
                self.statusMessage.set(" Diameter should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_ToolDIA_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_ToolDIA, self.Entry_ToolDIA_Check(), new=1)
    #############################
    def Entry_Vangle_Check(self):
        try:
            value = float(self.v_angle.get())
            if  value <= 0 or value >= 180:
                self.statusMessage.set(" Angle should be between 0 and 180")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_Vangle_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Vangle, self.Entry_Vangle_Check(), new=1)
    #############################
    def Entry_Feed_Check(self):
        try:
            value = float(self.f_feed.get())
            if  value <= 0.0:
                self.statusMessage.set(" Feed should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 1         # Value is a valid number changes do not require recalc
    def Entry_Feed_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Feed,self.Entry_Feed_Check(), new=1)
    #############################
    def Entry_p_feed_Check(self):
        try:
            value = float(self.p_feed.get())
            if  value <= 0.0:
                self.statusMessage.set(" Feed should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_p_feed_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_p_feed, self.Entry_p_feed_Check(), new=1)
    #############################
    def Entry_StepOver_Check(self):
        try:
            value = float(self.stepover.get())
            if  value <= 0.0:
                self.statusMessage.set(" Stepover should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_StepOver_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_StepOver, self.Entry_StepOver_Check(), new=1)
    #############################
    def Entry_Zsafe_Check(self):
        try:
            value = float(self.z_safe.get())
            if  value <= 0.0:
                self.statusMessage.set(" Z safe should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 1         # Value is a valid number changes do not require recalc
    def Entry_Zsafe_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Zsafe,self.Entry_Zsafe_Check(), new=1)
    #############################
    def Entry_Zcut_Check(self):
        try:
            value = float(self.z_cut.get())
            if  value >= 0.0:
                self.statusMessage.set(" Max depth should be less than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 1         # Value is a valid number changes do not require recalc
    def Entry_Zcut_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Zcut,self.Entry_Zcut_Check(), new=1)
    #############################
    # End Right Column #
    #############################

    #############################
    # Start ROUGH Setttings     #
    #############################
    
    def ROUGH_Entry_ToolDIA_Check(self):
        try:
            value = float(self.rough_dia.get())
            if  value <= 0.0:
                self.statusMessage.set(" Diameter should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number

    def ROUGH_Entry_ToolDIA_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_ToolDIA, self.ROUGH_Entry_ToolDIA_Check(), new=1)
    
    def ROUGH_Entry_Vangle_Check(self):
        try:
            value = float(self.rough_v_angle.get())
            if  value <= 0 or value >= 180:
                self.statusMessage.set(" Angle should be between 0 and 180")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def ROUGH_Entry_Vangle_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_Vangle, self.ROUGH_Entry_Vangle_Check(), new=1)
    
    def ROUGH_Entry_Feed_Check(self):
        try:
            value = float(self.rough_r_feed.get())
            if  value <= 0.0:
                self.statusMessage.set(" Feed should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 1         # Value is a valid number changes do not require recalc
    def ROUGH_Entry_Feed_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_Feed,self.ROUGH_Entry_Feed_Check(), new=1)
    
    def ROUGH_Entry_p_feed_Check(self):
        try:
            value = float(self.rough_p_feed.get())
            if  value <= 0.0:
                self.statusMessage.set(" Feed should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def ROUGH_Entry_p_feed_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_p_feed, self.ROUGH_Entry_p_feed_Check(), new=1)
    
    def ROUGH_Entry_StepOver_Check(self):
        try:
            value = float(self.rough_stepover.get())
            if  value <= 0.0:
                self.statusMessage.set(" Stepover should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def ROUGH_Entry_StepOver_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_StepOver, self.ROUGH_Entry_StepOver_Check(), new=1)
    
    def ROUGH_Entry_Roffset_Check(self):
        try:
            value = float(self.rough_offset.get())
            if  value < 0.0:
                self.statusMessage.set(" Roughing offset should be greater than or equal to 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def ROUGH_Entry_Roffset_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_Roffset, self.ROUGH_Entry_Roffset_Check(), new=1)
    
    def ROUGH_Entry_Rdepth_Check(self):
        try:
            value = float(self.rough_depth_pp.get())
            if  value < 0.0:
                self.statusMessage.set(" Roughing depth per pass should be greater than or equal to 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def ROUGH_Entry_Rdepth_Callback(self, varName, index, mode):
        self.entry_set(self.ROUGH_Entry_Rdepth, self.ROUGH_Entry_Rdepth_Check(), new=1)
    
    # End ROUGH setttings       #
    #############################

    
    def Entry_Tolerance_Check(self):
        try:
            value = float(self.tolerance.get())
            if  value <= 0.0:
                self.statusMessage.set(" Tolerance should be greater than 0 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_Tolerance_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_Tolerance,self.Entry_Tolerance_Check(), new=1)
    
    def Entry_ContAngle_Check(self):
        try:
            value = float(self.cangle.get())
            if  value <= 0.0 or value >= 90:
                self.statusMessage.set(" Contact angle should be between 0 and 90 ")
                return 2 # Value is invalid number
        except:
            return 3     # Value not a number
        return 0         # Value is a valid number
    def Entry_ContAngle_Callback(self, varName, index, mode):
        self.entry_set(self.Entry_ContAngle,self.Entry_ContAngle_Check(), new=1)
    #############################

    ##########################################################################
    ##########################################################################
    def Check_All_Variables(self):
        MAIN_error_cnt= \
        self.entry_set(self.Entry_Yscale, self.Entry_Yscale_Check()    ,2) +\
        self.entry_set(self.Entry_Toptol, self.Entry_Toptol_Check()    ,2) +\
        self.entry_set(self.Entry_ToolDIA, self.Entry_ToolDIA_Check()  ,2) +\
        self.entry_set(self.Entry_Vangle, self.Entry_Vangle_Check()    ,2) +\
        self.entry_set(self.Entry_Feed,self.Entry_Feed_Check()         ,2) +\
        self.entry_set(self.Entry_p_feed, self.Entry_p_feed_Check()  ,2)   +\
        self.entry_set(self.Entry_StepOver, self.Entry_StepOver_Check(),2) +\
        self.entry_set(self.Entry_Zsafe,self.Entry_Zsafe_Check()       ,2) +\
        self.entry_set(self.Entry_Zcut,self.Entry_Zcut_Check()         ,2)

        GEN_error_cnt= \
        self.entry_set(self.Entry_Tolerance,self.Entry_Tolerance_Check(),2) +\
        self.entry_set(self.Entry_ContAngle,self.Entry_ContAngle_Check(),2)
        
        ROUGH_error_cnt= \
        self.entry_set(self.ROUGH_Entry_ToolDIA, self.ROUGH_Entry_ToolDIA_Check()  ,2) +\
        self.entry_set(self.ROUGH_Entry_Vangle, self.ROUGH_Entry_Vangle_Check()    ,2) +\
        self.entry_set(self.ROUGH_Entry_Feed,self.ROUGH_Entry_Feed_Check()         ,2) +\
        self.entry_set(self.ROUGH_Entry_p_feed, self.ROUGH_Entry_p_feed_Check()  ,2)   +\
        self.entry_set(self.ROUGH_Entry_StepOver, self.ROUGH_Entry_StepOver_Check(),2) +\
        self.entry_set(self.ROUGH_Entry_Roffset, self.ROUGH_Entry_Roffset_Check()  ,2) +\
        self.entry_set(self.ROUGH_Entry_Rdepth, self.ROUGH_Entry_Rdepth_Check()    ,2)


        ERROR_cnt = MAIN_error_cnt + GEN_error_cnt + ROUGH_error_cnt

        if (ERROR_cnt > 0):
            self.statusbar.configure( bg = 'red' )
        if (GEN_error_cnt > 0):
            self.statusMessage.set(\
                " Entry Error Detected: Check Entry Values in General Settings Window ")
        if (MAIN_error_cnt > 0):
            self.statusMessage.set(\
                " Entry Error Detected: Check Entry Values in Main Window ")
        if (ROUGH_error_cnt > 0):
            self.statusMessage.set(\
                " Entry Error Detected: Check Entry Values in Roughing Settigns Window ")

        return ERROR_cnt

    def Entry_units_var_Callback(self):
        if (self.units.get() == 'in') and (self.funits.get()=='mm/min'):
            self.Scale_Linear_Inputs(1/25.4)
            self.funits.set('in/min')
        elif (self.units.get() == 'mm') and (self.funits.get()=='in/min'):
            self.Scale_Linear_Inputs(25.4)
            self.funits.set('mm/min')

    def Scale_Linear_Inputs(self, factor=1.0):
        try:
            self.yscale.set(        '%.3g' %(float(self.yscale.get()        )*factor) )
            self.toptol.set(        '%.3g' %(float(self.toptol.get()        )*factor) )
            self.dia.set(           '%.3g' %(float(self.dia.get()           )*factor) )
            self.f_feed.set(        '%.3g' %(float(self.f_feed.get()        )*factor) )
            self.p_feed.set(        '%.3g' %(float(self.p_feed.get()        )*factor) )
            self.stepover.set(      '%.3g' %(float(self.stepover.get()      )*factor) )
            self.z_cut.set(         '%.3g' %(float(self.z_cut.get()         )*factor) )
            self.z_safe.set(        '%.3g' %(float(self.z_safe.get()        )*factor) )
            self.rough_r_feed.set(  '%.3g' %(float(self.rough_r_feed.get()  )*factor) )
            self.rough_p_feed.set(  '%.3g' %(float(self.rough_p_feed.get()  )*factor) )
            self.rough_stepover.set('%.3g' %(float(self.rough_stepover.get())*factor) )
            self.rough_depth_pp.set('%.3g' %(float(self.rough_depth_pp.get())*factor) )
            self.rough_offset.set(  '%.3g' %(float(self.rough_offset.get()  )*factor) )
            self.rough_dia.set(     '%.3g' %(float(self.rough_dia.get()     )*factor) )
            self.tolerance.set(     '%.3g' %(float(self.tolerance.get()     )*factor) )
        except:
            pass

    def menu_File_Open_G_Code_File(self):
        init_dir = os.path.dirname(self.NGC_FILE)
        if ( not os.path.isdir(init_dir) ):
            init_dir = os.path.expanduser("~")
        fileselect = askopenfilename(filetypes=[("Gcode Files","*.ngc"),\
                                                ("TAP File","*.tap"),\
                                                ("All Files","*")],\
                                                 initialdir=init_dir)

        if fileselect != '' and fileselect != ():
            self.Open_G_Code_File(fileselect)

    def menu_File_Open_IMAGE_File(self):
        init_dir = os.path.dirname(self.IMAGE_FILE)
        if ( not os.path.isdir(init_dir) ):
            init_dir = os.path.expanduser("~")

        fileselect = askopenfilename(filetypes=[("Image Files",
                                     ("*.pgm","*.jpg","*.png","*.gif")),
                                     ("All Files","*")],
                                     initialdir=init_dir)

        if fileselect != '' and fileselect != ():
            self.Read_image_file(fileselect)
            self.Plot_Data()

    def Open_G_Code_File(self,filename):
        try:
            fin = open(filename,'r')
        except:
            fmessage("Unable to open file: %s" %(filename))
            return
        
        text_codes=[]
        ident = "cfg_set"
        for line in fin:
            if ident in line:
                print line
                # BOOL
                if   "show_axis"  in line:
                    self.show_axis.set(line[line.find("show_axis"):].split()[1])
                elif "invert"  in line:
                    self.invert.set(line[line.find("invert"):].split()[1])
                elif "normalize"  in line:
                    self.normalize.set(line[line.find("normalize"):].split()[1])
                elif "cuttop"  in line:
                    self.cuttop.set(line[line.find("cuttop"):].split()[1])
                elif "cutperim"  in line:
                    self.cutperim.set(line[line.find("cutperim"):].split()[1])
                elif "no_arcs"  in line:
                    self.no_arcs.set(line[line.find("no_arcs"):].split()[1])
                elif "grbl_flag"  in line:
                    self.grbl_flag.set(line[line.find("grbl_flag"):].split()[1])
                # STRING.set()
                elif "yscale"     in line:
                    self.yscale.set(line[line.find("yscale"):].split()[1])
                elif "toptol"    in line:
                    self.toptol.set(line[line.find("toptol"):].split()[1])
                elif "vangle"    in line:
                    self.v_angle.set(line[line.find("vangle"):].split()[1])
                elif "fh_stepover" in line:
                    self.stepover.set(line[line.find("fh_stepover"):].split()[1])
                elif "plfeed"    in line:
                    self.p_feed.set(line[line.find("plfeed"):].split()[1])
                elif "z_safe"    in line:
                    self.z_safe.set(line[line.find("z_safe"):].split()[1])
                elif "z_cut"    in line:
                    self.z_cut.set(line[line.find("z_cut"):].split()[1])
                elif "diatool"    in line:
                    self.dia.set(line[line.find("diatool"):].split()[1])
                elif "origin"    in line:
                    self.origin.set(line[line.find("origin"):].split()[1])
                elif "tool"    in line:
                    self.tool.set(line[line.find("tool"):].split()[1])
                elif "units"    in line:
                    self.units.set(line[line.find("units"):].split()[1])
                elif "plunge"    in line:
                    self.plungetype.set(line[line.find("plunge"):].split()[1])
                elif "fh_r_feed"    in line:
                     self.f_feed.set(line[line.find("fh_r_feed"):].split()[1])
                elif "lace"    in line:
                     self.lace_bound.set(line[line.find("lace"):].split()[1])
                elif "cangle"    in line:
                     self.cangle.set(line[line.find("cangle"):].split()[1])    
                elif "tolerance"    in line:
                     self.tolerance.set(line[line.find("tolerance"):].split()[1])
                elif "splitstep"    in line:
                     self.splitstep.set(line[line.find("splitstep"):].split()[1])

                elif "fh_scanpat"    in line:
                     self.scanpat.set(line[line.find("fh_scanpat"):].split("\042")[1])
                elif "fh_scandir"    in line:
                     self.scandir.set(line[line.find("fh_scandir"):].split("\042")[1])
                elif "gpre"    in line:
                     self.gpre.set(line[line.find("gpre"):].split("\042")[1])
                elif "gpost"    in line:
                     self.gpost.set(line[line.find("gpost"):].split("\042")[1])
                elif "imagefile"    in line:
                       self.IMAGE_FILE=(line[line.find("imagefile"):].split("\042")[1])
                       
                elif "rough_tool"    in line:
                     self.rough_tool.set(line[line.find("rough_tool"):].split()[1])
                elif "rough_dia"    in line:
                     self.rough_dia.set(line[line.find("rough_dia"):].split()[1])
                elif "rough_v_angle"    in line:
                     self.rough_v_angle.set(line[line.find("rough_v_angle"):].split()[1])
                elif "rough_r_feed"    in line:
                     self.rough_r_feed.set(line[line.find("rough_r_feed"):].split()[1])
                elif "rough_p_feed"    in line:
                     self.rough_p_feed.set(line[line.find("rough_p_feed"):].split()[1])
                elif "rough_stepover"    in line:
                     self.rough_stepover.set(line[line.find("rough_stepover"):].split()[1])
                elif "rough_depth_pp"    in line:
                     self.rough_depth_pp.set(line[line.find("rough_depth_pp"):].split()[1])
                elif "rough_offset"    in line:
                     self.rough_offset.set(line[line.find("rough_offset"):].split()[1])                     
                elif "rough_scanpat"    in line:
                     self.rough_scanpat.set(line[line.find("rough_scanpat"):].split("\042")[1])
                elif "rough_scandir"    in line:
                     self.rough_scandir.set(line[line.find("rough_scandir"):].split("\042")[1])
                     
        fin.close()
            
        fileName, fileExtension = os.path.splitext(self.IMAGE_FILE)
        init_file=os.path.basename(fileName)
        if init_file != "None":
            if ( os.path.isfile(self.IMAGE_FILE) ):
                self.Read_image_file(self.IMAGE_FILE)
            else:
                self.statusMessage.set("Image file not found: %s " %(self.IMAGE_FILE))

        if self.units.get() == 'in':
            self.funits.set('in/min')
        else:
            self.units.set('mm')
            self.funits.set('mm/min')

        temp_name, fileExtension = os.path.splitext(filename)
        file_base=os.path.basename(temp_name)
            
        if self.initComplete == 1:
            self.menu_Mode_Change()
            self.NGC_FILE = filename

    def Read_image_file(self,fileselect):
        im = []
        if not ( os.path.isfile(fileselect) ):
            self.statusMessage.set("Image file not found: %s" %(fileselect))
            self.statusbar.configure( bg = 'red' )            
        else:
            self.statusMessage.set("Image file: %s " %(fileselect))
            self.statusbar.configure( bg = 'white' ) 
            try:
                PIL_im = Image.open(fileselect)
                self.wim, self.him = PIL_im.size
                # Convert image to grayscale
                PIL_im = PIL_im.convert("L") 

                self.aspect_ratio =  float(self.wim-1) / float(self.him-1)
                self.Xscale.set("%.3f" %( self.aspect_ratio * float(self.yscale.get())) ) 
                self.pixsize.set("%.3f" %( float(self.yscale.get()) / (self.him - 1.0) ) )
                                
                self.im = PIL_im
                self.SCALE = 1
                self.ui_TKimage = ImageTk.PhotoImage(self.im.resize((50,50), Image.ANTIALIAS))
                self.IMAGE_FILE = fileselect
                    
            except:
                self.statusMessage.set("Unable to Open Image file: %s" %(self.IMAGE_FILE))
                self.statusbar.configure( bg = 'red' )    
                
    def menu_File_Save_G_Code_File_Finish(self):
        self.menu_File_Save_G_Code_File(rough_flag = 0)

    def menu_File_Save_G_Code_File_Rough(self):
        self.menu_File_Save_G_Code_File(rough_flag = 1)

    def menu_File_Save_G_Code_File(self,rough_flag = 0):
        global STOP_CALC
        STOP_CALC = 0
        if (self.Check_All_Variables() > 0):
            return
        
        init_dir = os.path.dirname(self.NGC_FILE)
        if ( not os.path.isdir(init_dir) ):
            init_dir = os.path.expanduser("~")

        fileName, fileExtension = os.path.splitext(self.NGC_FILE)
        init_file=os.path.basename(fileName)
        
        fileName, fileExtension = os.path.splitext(self.IMAGE_FILE)
        init_file=os.path.basename(fileName)


        init_file = init_file.replace('_rough', '')
        if rough_flag == 1:
            init_file = init_file + "_rough"
        filename = asksaveasfilename(defaultextension='.ngc', \
                                     filetypes=[("G-Code Files","*.ngc"),("TAP File","*.tap"),("All Files","*")],\
                                     initialdir=init_dir,\
                                     initialfile= init_file )
        
        if filename != '' and filename != ():
            self.NGC_FILE = filename

            try:
                fout = open(filename,'w')
            except:
                self.statusMessage.set("Unable to open file for writing: %s" %(filename))
                self.statusbar.configure( bg = 'red' )
                return

            vcalc_status = Toplevel(width=525, height=50)
            # Use grab_set to prevent user input in the main window during calculations
            vcalc_status.grab_set()

            self.statusbar2 = Label(vcalc_status, textvariable=self.statusMessage, bd=1, relief=FLAT , height=1)
            self.statusbar2.place(x=130+12+12, y=12, width=350, height=30)
            self.statusMessage.set("Preparing Image Data")
            self.statusbar.configure( bg = 'yellow' )

            STOP_CALC = 0
            self.stop_button = Button(vcalc_status,text="Stop Calculation")
            self.stop_button.place(x=12, y=12, width=130, height=30)
            self.stop_button.bind("<ButtonRelease-1>", self.Stop_Click)
            
            vcalc_status.resizable(0,0)
            vcalc_status.title('Saving File')
            vcalc_status.iconname(p_name)
            
            try: #Attempt to create temporary icon bitmap file
                f = open(p_name + "_icon",'w')
                f.write("#define " + p_name + "_icon_width 16\n")
                f.write("#define " + p_name + "_icon_height 16\n")
                f.write("static unsigned char " + p_name + "_icon_bits[] = {\n")
                f.write("   0x3f, 0xfc, 0x1f, 0xf8, 0xcf, 0xf3, 0x6f, 0xe4, 0x6f, 0xed, 0xcf, 0xe5,\n")
                f.write("   0x1f, 0xf4, 0xfb, 0xf3, 0x73, 0x98, 0x47, 0xce, 0x0f, 0xe0, 0x3f, 0xf8,\n")
                f.write("   0x7f, 0xfe, 0x3f, 0xfc, 0x9f, 0xf9, 0xcf, 0xf3 };\n")
                f.close()
                vcalc_status.iconbitmap("@" + p_name + "_icon")
                os.remove(p_name + "_icon")
            except:
                fmessage("Unable to create temporary icon file.")

            vcalc_status.update_idletasks()
            self.WriteGCode(rough_flag = rough_flag)
            for line in self.gcode:
                try:
                    fout.write(line+'\n')
                except:
                    fmessage("skipping g-code line:" + line + "; may be due to non ASCII character.");
                    pass
            fout.close
            if STOP_CALC == 0:
                self.statusMessage.set("File Saved: %s" %(filename))
                self.statusbar.configure( bg = 'white' )
            else:
                self.statusMessage.set("File Save Terminated")
                self.statusbar.configure( bg = 'yellow' )
            vcalc_status.grab_release()
            try:
                vcalc_status.destroy()
            except:
                pass
    ################
    ################
            
    def menu_File_Quit(self):
        if message_ask_ok_cancel("Exit", "Exiting...."):
            self.Quit_Click(None)

    def menu_View_Refresh_Callback(self, varName, index, mode):
        self.menu_View_Refresh()

    def menu_View_Refresh(self):
        dummy_event = Event()
        dummy_event.widget=self.master
        self.Master_Configure(dummy_event,1)
        self.Plot_Data()

    def menu_Mode_Change_Callback(self, varName, index, mode):
        self.menu_View_Refresh()

    def menu_Mode_Change(self):
        dummy_event = Event()
        dummy_event.widget=self.master
        self.Master_Configure(dummy_event,1)

    def menu_View_Recalculate(self):
        pass

    def menu_Help_About(self):
        about = " " + p_name + " by Onekk.\n"
        about = about + " Fork of the excellent "
        about = about + " dmap2gcode by Scorch"       
        message_box("About " + p_name, about)

    def KEY_ESC(self, event):
        pass #A stop calculation command may go here

    def KEY_F1(self, event):
        self.menu_Help_About()

    def KEY_F2(self, event):
        self.GEN_Settings_Window()

    def KEY_F3(self, event):
        self.ROUGH_Settings_Window()

    def KEY_F4(self, event):
        pass

    def KEY_F5(self, event):
        self.menu_View_Refresh()

    def KEY_CTRL_G(self, event):
        self.CopyClipboard_GCode()

    def bindConfigure(self, event):
        if not self.initComplete:
            self.initComplete = 1
            self.menu_Mode_Change()

    def Master_Configure(self, event, update=0):
        if event.widget != self.master:
            return
        x = int(self.master.winfo_x())
        y = int(self.master.winfo_y())
        w = int(self.master.winfo_width())
        h = int(self.master.winfo_height())
        if (self.x, self.y) == (-1,-1):
            self.x, self.y = x,y
            
        if abs(self.w-w)>10 or abs(self.h-h)>10 or update==1:
            ######################
            #  Form changed Size (resized) adjust as required #
            ######################
            self.w = w
            self.h = h

            if 0 == 0:                
                # Left Column #
                w_label=90
                w_entry=60
                w_units=35

                x_label_L=10
                x_entry_L=x_label_L+w_label+10
                x_units_L=x_entry_L+w_entry+5

                Yloc=6
                self.Label_font_prop.place(x=x_label_L, y=Yloc, width=w_label*2, height=21)
                Yloc=Yloc+24
                self.Label_Yscale.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Label_Yscale_u.place(x=x_units_L, y=Yloc, width=w_units, height=21)
                self.Entry_Yscale.place(x=x_entry_L, y=Yloc, width=w_entry, height=23)

                Yloc=Yloc+24
                self.Label_Yscale2.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Label_Yscale2_u.place(x=x_units_L, y=Yloc, width=w_units, height=21)
                self.Label_Yscale2_val.place(x=x_entry_L, y=Yloc, width=w_entry, height=21)
                
                Yloc=Yloc+24
                self.Label_PixSize.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Label_PixSize_u.place(x=x_units_L, y=Yloc, width=w_units, height=21)
                self.Label_PixSize_val.place(x=x_entry_L, y=Yloc, width=w_entry, height=21)

                Yloc=Yloc+24+12
                self.separator1.place(x=x_label_L, y=Yloc,width=w_label+75+40, height=2)
                Yloc=Yloc+6
                self.Label_pos_orient.place(x=x_label_L, y=Yloc, width=w_label*2, height=21)

                Yloc=Yloc+24
                self.Label_Invert_Color_FALSE.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Radio_Invert_Color_FALSE.place(x=x_entry_L+20, y=Yloc, width=75, height=23)
                
                Yloc=Yloc+24
                #self.Label_Invert_Color_TRUE.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Radio_Invert_Color_TRUE.place(x=x_entry_L+20, y=Yloc, width=75, height=23)

                Yloc=Yloc+24
                self.Label_normalize.place(x=x_label_L, y=Yloc, width=w_label+20, height=21)
                self.CB_normalize.place(x=x_entry_L+20, y=Yloc, width=w_entry+20, height=23)

                Yloc=Yloc+24
                self.Label_Origin.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Origin_OptionMenu.place(x=x_entry_L, y=Yloc, width=w_entry+40, height=23)

                Yloc=Yloc+24+12
                self.separator2.place(x=x_label_L, y=Yloc,width=w_label+75+40, height=2)

                Yloc=Yloc+6
                self.Label_CutTop.place(x=x_label_L, y=Yloc, width=w_label+20, height=21)
                self.CB_CutTop.place(x=x_entry_L+20, y=Yloc, width=w_entry+20, height=23)
                
                Yloc=Yloc+24
                self.Label_Toptol.place(x=x_label_L, y=Yloc, width=w_label, height=21)
                self.Label_Toptol_u.place(x=x_units_L, y=Yloc, width=w_units, height=21)
                self.Entry_Toptol.place(x=x_entry_L, y=Yloc, width=w_entry, height=23)
                # End Left Column #


                # Start Right Column
                w_label=100
                w_entry=60
                w_units=40

                x_label_R=self.w - 500
                x_entry_R=x_label_R+w_label+10
                x_units_R=x_entry_R+w_entry+5

                Yloc=6
                self.Label_tool_opt.place(x=x_label_R, y=Yloc, width=w_label*2, height=21)

                Yloc=Yloc+24
                self.Label_ToolDIA.place(x=x_label_R,   y=Yloc, width=w_label, height=21)
                self.Label_ToolDIA_u.place(x=x_units_R, y=Yloc, width=w_units, height=21)
                self.Entry_ToolDIA.place(x=x_entry_R,   y=Yloc, width=w_entry, height=23)

                Yloc=Yloc+24
                self.Label_Tool.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Tool_OptionMenu.place(x=x_entry_R, y=Yloc, width=w_entry+40, height=23)

                Yloc=Yloc+24
                self.Label_Vangle.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Entry_Vangle.place(x=x_entry_R, y=Yloc, width=w_entry, height=23)
                
                Yloc=Yloc+24+12
                self.separator3.place(x=x_label_R, y=Yloc,width=w_label+75+40, height=2)

                Yloc=Yloc+6
                self.Label_gcode_opt.place(x=x_label_R, y=Yloc, width=w_label*2, height=21)

                Yloc=Yloc+24
                self.Label_Scanpat.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.ScanPat_OptionMenu.place(x=x_entry_R, y=Yloc, width=w_entry+40, height=23)
               
                Yloc=Yloc+24
                self.Label_Scandir.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.ScanDir_OptionMenu.place(x=x_entry_R, y=Yloc, width=w_entry+40, height=23)
                
                Yloc=Yloc+24
                self.Entry_Feed.place(  x=x_entry_R, y=Yloc, width=w_entry, height=23)
                self.Label_Feed.place(  x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Label_Feed_u.place(x=x_units_R, y=Yloc, width=w_units+15, height=21)

                Yloc=Yloc+24
                self.Label_p_feed.place(x=x_label_R,  y=Yloc, width=w_label,   height=21)
                self.Entry_p_feed.place(x=x_entry_R,  y=Yloc, width=w_entry,   height=23)
                self.Label_p_feed_u.place(x=x_units_R,y=Yloc, width=w_units+15,height=21)

                Yloc=Yloc+24
                self.Label_StepOver.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Label_StepOver_u.place(x=x_units_R, y=Yloc, width=w_units, height=21)
                self.Entry_StepOver.place(x=x_entry_R, y=Yloc, width=w_entry, height=23)

                Yloc=Yloc+24
                self.Entry_Zsafe.place(  x=x_entry_R, y=Yloc, width=w_entry, height=23)
                self.Label_Zsafe.place(  x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Label_Zsafe_u.place(x=x_units_R, y=Yloc, width=w_units, height=21)


                Yloc=Yloc+24
                self.Label_Zcut.place(  x=x_label_R, y=Yloc, width=w_label, height=21)
                self.Label_Zcut_u.place(x=x_units_R, y=Yloc, width=w_units, height=21)
                self.Entry_Zcut.place(  x=x_entry_R, y=Yloc, width=w_entry, height=23)

                Yloc=Yloc+24
                self.Label_CutPerim.place(x=x_label_R, y=Yloc, width=w_label, height=21)
                self.CB_CutPerim.place(x=x_entry_R, y=Yloc, width=w_entry+40, height=23)

                Yloc=Yloc+24+12
                self.separator4.place(x=x_label_R, y=Yloc,width=w_label+75+40, height=2)

                # Buttons etc.
                Yloc = Yloc + 50
                self.Save_Button.place(x=x_label_R, y=Yloc, width=95, height=30)

                ## third column -Roughing settings

                x_label_3 = self.w - 260
                x_entry_3 = x_label_3 + w_label + 10
                x_units_3 = x_entry_3 + w_entry + 5                

                #self.ROUGH_sep1.place(x=x_label_3-3, y=50)
                
                D_Yloc=6
                self.ROUGH_Label_tool_opt.place(x=x_label_3, y=D_Yloc, width=w_label*2, height=21)


                D_Yloc=D_Yloc+24
                self.ROUGH_Label_ToolDIA.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Label_ToolDIA_u.place(x=x_units_3, y=D_Yloc, width=w_units, height=21)
                self.ROUGH_Entry_ToolDIA.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Tool.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.rough_tool_OptionMenu.place(x=x_entry_3, y=D_Yloc, width=w_entry+40, height=23)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Vangle.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Entry_Vangle.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)

                D_Yloc=D_Yloc+24+12
                self.ROUGH_sep3.place(x=x_label_3, y=D_Yloc,width=w_label+75+40, height=2)

                D_Yloc=D_Yloc+6        
                self.ROUGH_Label_gcode_opt.place(x=x_label_3, y=D_Yloc, width=w_label*2, height=21)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Scanpat.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_ScanPat_OptionMenu.place(x=x_entry_3, y=D_Yloc, width=w_entry+40, height=23)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Scandir.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_ScanDir_OptionMenu.place(x=x_entry_3, y=D_Yloc, width=w_entry+40, height=23)

                D_Yloc=D_Yloc+24
                self.ROUGH_Entry_Feed.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)
                self.ROUGH_Label_Feed.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Label_Feed_u.place(x=x_units_3, y=D_Yloc, width=w_units+15, height=21)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_p_feed.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Entry_p_feed.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)
                self.ROUGH_Label_p_feed_u.place(x=x_units_3, y=D_Yloc, width=w_units+15,height=21)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_StepOver.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Label_StepOver_u.place(x=x_units_3, y=D_Yloc, width=w_units, height=21)
                self.ROUGH_Entry_StepOver.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)

                D_Yloc=D_Yloc+24+12
                self.ROUGH_sep4.place(x=x_label_3, y=D_Yloc,width=w_label+75+40, height=2)

                D_Yloc=D_Yloc+6
                self.ROUGH_Label_roughing_props.place(x=x_label_3, y=D_Yloc, width=w_label*2, height=21)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Roffset.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Label_Roffset_u.place(x=x_units_3, y=D_Yloc, width=w_units, height=21)
                self.ROUGH_Entry_Roffset.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)

                D_Yloc=D_Yloc+24
                self.ROUGH_Label_Rdepth.place(x=x_label_3, y=D_Yloc, width=w_label, height=21)
                self.ROUGH_Label_Rdepth_u.place(x=x_units_3, y=D_Yloc, width=w_units, height=21)
                self.ROUGH_Entry_Rdepth.place(x=x_entry_3, y=D_Yloc, width=w_entry, height=23)

                D_Yloc=D_Yloc+24+12
                self.ROUGH_sep2.place(x=x_label_3, y=D_Yloc,width=w_label+75+40, height=2)

                D_Yloc=D_Yloc+50
                self.ROUGH_Save.place(x=x_label_3, y=D_Yloc, width=130, height=40) 
               
                self.PreviewCanvas.configure(width=self.canvas_width, height=self.h-100)
                self.PreviewCanvas_frame.place(x=220, y=20)

                self.Set_Input_States()
                self.Set_Input_States_ROUGH()
                
            self.Plot_Data()
            
    def Recalculate_RQD_Click(self, event):
        self.menu_View_Refresh()

    def Set_Input_States(self):
        if self.tool.get() != "V":
            self.Label_Vangle.configure(state="disabled")
            self.Entry_Vangle.configure(state="disabled")
        else:
            self.Label_Vangle.configure(state="normal")
            self.Entry_Vangle.configure(state="normal")

        if self.cuttop.get():
            self.Entry_Toptol.configure(state="disabled")
            self.Label_Toptol.configure(state="disabled")
            self.Label_Toptol_u.configure(state="disabled")
        else:
            self.Entry_Toptol.configure(state="normal")
            self.Label_Toptol.configure(state="normal")
            self.Label_Toptol_u.configure(state="normal")
            
    def Set_Input_States_Event(self,event):
        self.Set_Input_States()

    def Set_Input_States_GEN(self):
        if self.lace_bound.get() == "None":
            self.Label_ContAngle.configure(state="disabled")
            self.Entry_ContAngle.configure(state="disabled")
        else:
            self.Label_ContAngle.configure(state="normal")
            self.Entry_ContAngle.configure(state="normal")

        if ( self.scanpat.get().find("R") == -1) or \
           ( self.scanpat.get().find("C") == -1):
            self.Label_LaceBound.configure(state="disabled")
            self.LaceBound_OptionMenu.configure(state="disabled")
            self.Label_ContAngle.configure(state="disabled")
            self.Entry_ContAngle.configure(state="disabled")
        else:
            self.Label_LaceBound.configure(state="normal")
            self.LaceBound_OptionMenu.configure(state="normal")
            
    def Set_Input_States_GEN_Event(self,event):
        self.Set_Input_States_GEN()

    def Set_Input_States_ROUGH(self):
        if self.rough_tool.get() != "V":
            self.ROUGH_Label_Vangle.configure(state="disabled")
            self.ROUGH_Entry_Vangle.configure(state="disabled")
        else:
            self.ROUGH_Label_Vangle.configure(state="normal")
            self.ROUGH_Entry_Vangle.configure(state="normal")
            
    def Set_Input_States_Event_ROUGH(self,event):
        self.Set_Input_States_ROUGH()
        
    def Plot_Data(self):
        """\
        CANVAS PLOTTING STUFF     
        """
        self.PreviewCanvas.delete(ALL)
        
        if (self.Check_All_Variables() > 0):
            return
        
        cszw = int(self.PreviewCanvas.cget("width"))
        cszh = int(self.PreviewCanvas.cget("height"))
        wc = float(cszw/2)
        hc = float(cszh/2)

        try:
            test = self.im.size
            self.SCALE = min( float(cszw-20)/float(self.wim), float(cszh-20)/float(self.him))
            if self.SCALE < 1:
                nw=int(self.SCALE*self.wim)
                nh=int(self.SCALE*self.him)
            else:
                nw = self.wim
                nh = self.him
                self.SCALE = 1
            self.ui_TKimage = ImageTk.PhotoImage(self.im.resize((nw,nh), Image.ANTIALIAS))
        except:
            self.SCALE = 1            

        self.canvas_image = self.PreviewCanvas.create_image(wc, \
                            hc, anchor=CENTER, image=self.ui_TKimage)

        midx = 0
        midy = 0
        minx = int(self.wim/2)
        miny = int(self.him/2)
        maxx = -minx
        maxy = -miny
        
        # ORIGIN LOCATING STUFF

        CASE = str(self.origin.get())
        if     CASE == "Top-Left":
            x_zero = minx
            y_zero = maxy
        elif   CASE == "Top-Center":
            x_zero = midx
            y_zero = maxy
        elif   CASE == "Top-Right":
            x_zero = maxx
            y_zero = maxy
        elif   CASE == "Mid-Left":
            x_zero = minx
            y_zero = midy
        elif   CASE == "Mid-Center":
            x_zero = midx
            y_zero = midy
        elif   CASE == "Mid-Right":
            x_zero = maxx
            y_zero = midy
        elif   CASE == "Bot-Left":
            x_zero = minx
            y_zero = miny
        elif   CASE == "Bot-Center":
            x_zero = midx
            y_zero = miny
        elif   CASE == "Bot-Right":
            x_zero = maxx
            y_zero = miny
        else:          #"Default"
            x_zero = minx
            y_zero = miny    
        
        axis_length = int(self.wim/4)

        PlotScale =  self.SCALE
        axis_x1 =  cszw/2 + (-x_zero             ) * PlotScale
        axis_x2 =  cszw/2 + ( axis_length-x_zero ) * PlotScale
        axis_y1 =  cszh/2 - (-y_zero             ) * PlotScale
        axis_y2 =  cszh/2 - ( axis_length-y_zero ) * PlotScale
        
        for seg in self.segID:
            self.PreviewCanvas.delete(seg)
        self.segID = []
        if self.show_axis.get() == True:
            # Plot coordinate system origin
            self.segID.append(self.PreviewCanvas.create_line(axis_x1,axis_y1,\
                                                             axis_x2,axis_y1,\
                                                             fill = 'red',
                                                              width = 2))
            self.segID.append(self.PreviewCanvas.create_line(axis_x1,axis_y1,\
                                                             axis_x1,axis_y2,\
                                                             fill = 'green',
                                                             width = 2))

    def GEN_Settings_Window(self):
        gen_settings = Toplevel(width=560, height=360)
        gen_settings.grab_set() # Use grab_set to prevent user input in the main window during calculations
        gen_settings.resizable(0,0)
        gen_settings.title('Settings')
        gen_settings.iconname("Settings")

        try: #Attempt to create temporary icon bitmap file
            f = open(p_name + "_icon",'w')
            f.write("#define " + p_name + "_icon_width 16\n")
            f.write("#define " + p_name + "_icon_height 16\n")
            f.write("static unsigned char " + p_name + "_icon_bits[] = {\n")
            f.write("   0x3f, 0xfc, 0x1f, 0xf8, 0xcf, 0xf3, 0x6f, 0xe4, 0x6f, 0xed, 0xcf, 0xe5,\n")
            f.write("   0x1f, 0xf4, 0xfb, 0xf3, 0x73, 0x98, 0x47, 0xce, 0x0f, 0xe0, 0x3f, 0xf8,\n")
            f.write("   0x7f, 0xfe, 0x3f, 0xfc, 0x9f, 0xf9, 0xcf, 0xf3 };\n")
            f.close()
            gen_settings.iconbitmap("@" + p_name + "_icon")
            os.remove(p_name + "_icon")
        except:
            pass

        D_Yloc  = 6
        D_dY = 24
        xd_label_L = 12

        w_label=110
        w_entry=60
        w_units=35
        xd_entry_L=xd_label_L+w_label+10
        xd_units_L=xd_entry_L+w_entry+5

        #Radio Button
        D_Yloc=D_Yloc+D_dY
        self.Label_Units = Label(gen_settings,text="Units")
        self.Label_Units.place(x=xd_label_L, y=D_Yloc, width=113, height=21)
        self.Radio_Units_IN = Radiobutton(gen_settings,text="inch", value="in",
                                         width="100", anchor=W)
        self.Radio_Units_IN.place(x=w_label+22, y=D_Yloc, width=75, height=23)
        self.Radio_Units_IN.configure(variable=self.units, command=self.Entry_units_var_Callback )
        self.Radio_Units_MM = Radiobutton(gen_settings,text="mm", value="mm",
                                         width="100", anchor=W)
        self.Radio_Units_MM.place(x=w_label+110, y=D_Yloc, width=75, height=23)
        self.Radio_Units_MM.configure(variable=self.units, command=self.Entry_units_var_Callback )

        D_Yloc=D_Yloc+D_dY
        self.Label_Tolerance = Label(gen_settings,text="tolerance")
        self.Label_Tolerance.place(x=xd_label_L, y=D_Yloc, width=w_label, height=21)
        self.Label_Tolerance_u = Label(gen_settings,textvariable=self.units, anchor=W)
        self.Label_Tolerance_u.place(x=xd_units_L, y=D_Yloc, width=w_units, height=21)
        self.Entry_Tolerance = Entry(gen_settings,width="15")
        self.Entry_Tolerance.place(x=xd_entry_L, y=D_Yloc, width=w_entry, height=23)
        self.Entry_Tolerance.configure(textvariable=self.tolerance)
        self.tolerance.trace_variable("w", self.Entry_Tolerance_Callback)
        self.entry_set(self.Entry_Tolerance,self.Entry_Tolerance_Check(),2)

        D_Yloc=D_Yloc+D_dY
        self.Label_Gpre = Label(gen_settings,text="G Code Header")
        self.Label_Gpre.place(x=xd_label_L, y=D_Yloc, width=w_label, height=21)
        self.Entry_Gpre = Entry(gen_settings,width="15")
        self.Entry_Gpre.place(x=xd_entry_L, y=D_Yloc, width=300, height=23)
        self.Entry_Gpre.configure(textvariable=self.gpre)

        D_Yloc=D_Yloc+D_dY
        self.Label_Gpost = Label(gen_settings,text="G Code Postamble")
        self.Label_Gpost.place(x=xd_label_L, y=D_Yloc, width=w_label, height=21)
        self.Entry_Gpost = Entry(gen_settings)
        self.Entry_Gpost.place(x=xd_entry_L, y=D_Yloc, width=300, height=23)
        self.Entry_Gpost.configure(textvariable=self.gpost)
        
        D_Yloc=D_Yloc+D_dY
        self.Label_LaceBound = Label(gen_settings,text="Lace Bounding")
        self.Label_LaceBound.place(x=xd_label_L, y=D_Yloc, width=w_label, height=21)
        self.LaceBound_OptionMenu = OptionMenu(gen_settings, self.lace_bound, "None","Secondary","Full",\
                                               command=self.Set_Input_States_GEN_Event)
        self.LaceBound_OptionMenu.place(x=xd_entry_L, y=D_Yloc, width=w_entry+40, height=23)

        D_Yloc=D_Yloc+D_dY
        self.Label_ContAngle = Label(gen_settings,text="LB Contact Angle")
        self.Label_ContAngle.place(x=xd_label_L, y=D_Yloc, width=w_label, height=21)
        self.Label_ContAngle_u = Label(gen_settings,text="deg", anchor=W)
        self.Label_ContAngle_u.place(x=xd_units_L, y=D_Yloc, width=w_units, height=21)
        self.Entry_ContAngle = Entry(gen_settings,width="15")
        self.Entry_ContAngle.place(x=xd_entry_L, y=D_Yloc, width=w_entry, height=23)
        self.Entry_ContAngle.configure(textvariable=self.cangle)
        self.cangle.trace_variable("w", self.Entry_ContAngle_Callback)
        self.entry_set(self.Entry_ContAngle,self.Entry_ContAngle_Check(),2)

        #Radio Button
        D_Yloc=D_Yloc+D_dY
        self.Label_SplitStep = Label(gen_settings,text="Offset Stepover")
        self.Label_SplitStep.place(x=xd_label_L, y=D_Yloc, width=113, height=21)


        self.Radio_SplitStep_N = Radiobutton(gen_settings,text="None", value="0",
                                         width="100", anchor=W)
        self.Radio_SplitStep_N.place(x=w_label+22, y=D_Yloc, width=75, height=23)
        self.Radio_SplitStep_N.configure(variable=self.splitstep )

        self.Radio_SplitStep_H = Radiobutton(gen_settings,text="1/2 Step", value="0.5",
                                         width="100", anchor=W)
        self.Radio_SplitStep_H.place(x=w_label+110, y=D_Yloc, width=75, height=23)
        self.Radio_SplitStep_H.configure(variable=self.splitstep )

        self.Radio_SplitStep_Q = Radiobutton(gen_settings,text="1/4 Step", value="0.25",
                                         width="100", anchor=W)
        self.Radio_SplitStep_Q.place(x=w_label+198, y=D_Yloc, width=75, height=23)
        self.Radio_SplitStep_Q.configure(variable=self.splitstep )

        #Radio Button
        D_Yloc=D_Yloc+D_dY
        self.Label_PlungeType = Label(gen_settings,text="Plunge Type")
        self.Label_PlungeType.place(x=xd_label_L, y=D_Yloc, width=113, height=21)
        self.Radio_PlungeType_S = Radiobutton(gen_settings,text="Vertical", value="simple",
                                         width="100", anchor=W)
        self.Radio_PlungeType_S.place(x=w_label+22, y=D_Yloc, width=75, height=23)
        self.Radio_PlungeType_S.configure(variable=self.plungetype )
        self.Radio_PlungeType_A = Radiobutton(gen_settings,text="Arc", value="arc",
                                         width="100", anchor=W)
        self.Radio_PlungeType_A.place(x=w_label+110, y=D_Yloc, width=75, height=23)
        self.Radio_PlungeType_A.configure(variable=self.plungetype )

        D_Yloc=D_Yloc+D_dY
        self.Label_Disable_Arcs = Label(gen_settings,text="Disable G-Code Arcs")
        self.Label_Disable_Arcs.place(x=xd_label_L, y=D_Yloc, width=113, height=21)
        self.CB_Disable_Arcs = Checkbutton(gen_settings,text=" ", \
                                              anchor=W, command=self.Set_Input_States)
        self.CB_Disable_Arcs.place(x=w_label+22, y=D_Yloc, width=75, height=23)
        
        self.Label_Disable_Arcs.place(x=xd_label_L, y=D_Yloc, width=113, height=21)
        self.CB_Disable_Arcs.configure(variable=self.no_arcs)
        
        ## Buttons ##
        gen_settings.update_idletasks()
        Ybut=int(gen_settings.winfo_height())-30
        Xbut=int(gen_settings.winfo_width()/2)

        self.GEN_Close = Button(gen_settings,text="Close",command=self.Close_Current_Window_Click)
        self.GEN_Close.place(x=Xbut, y=Ybut, width=130, height=30, anchor="center")

        self.Set_Input_States_GEN()

  
########################################
# Author.py                            #
# A component of emc2                  #
########################################

# Compute the 3D distance from the line segment l1..l2 to the point p.
# (Those are lower case L1 and L2)

def dist_lseg(l1, l2, p):
    x0, y0, z0 = l1
    xa, ya, za = l2
    xi, yi, zi = p

    dx = xa-x0
    dy = ya-y0
    dz = za-z0
    d2 = dx*dx + dy*dy + dz*dz

    if d2 == 0: return 0

    t = (dx * (xi-x0) + dy * (yi-y0) + dz * (zi-z0)) / d2
    if t < 0: t = 0
    if t > 1: t = 1
    dist2 = (xi - x0 - t*dx)**2 + (yi - y0 - t*dy)**2 + (zi - z0 - t*dz)**2

    return dist2 ** .5

def rad1(x1,y1,x2,y2,x3,y3):
    x12 = x1-x2
    y12 = y1-y2
    x23 = x2-x3
    y23 = y2-y3
    x31 = x3-x1
    y31 = y3-y1

    den = abs(x12 * y23 - x23 * y12)
    if abs(den) < 1e-5: return MAXINT
    return hypot(float(x12), float(y12)) * hypot(float(x23), float(y23)) * hypot(float(x31), float(y31)) / 2 / den

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self): return "<%f,%f>" % (self.x, self.y)
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    def __mul__(self, other):
        return Point(self.x * other, self.y * other)
    __rmul__ = __mul__
    def cross(self, other):
        return self.x * other.y - self.y * other.x
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    def mag(self):
        return hypot(self.x, self.y)
    def mag2(self):
        return self.x**2 + self.y**2

def cent1(x1,y1,x2,y2,x3,y3):
    P1 = Point(x1,y1)
    P2 = Point(x2,y2)
    P3 = Point(x3,y3)

    den = abs((P1-P2).cross(P2-P3))
    if abs(den) < 1e-5: return MAXINT, MAXINT

    alpha = (P2-P3).mag2() * (P1-P2).dot(P1-P3) / 2 / den / den
    beta  = (P1-P3).mag2() * (P2-P1).dot(P2-P3) / 2 / den / den
    gamma = (P1-P2).mag2() * (P3-P1).dot(P3-P2) / 2 / den / den

    Pc = alpha * P1 + beta * P2 + gamma * P3
    return Pc.x, Pc.y

def arc_center(plane, p1, p2, p3):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3

    if plane == 17: return cent1(x1,y1,x2,y2,x3,y3)
    if plane == 18: return cent1(x1,z1,x2,z2,x3,z3)
    if plane == 19: return cent1(y1,z1,y2,z2,y3,z3)

def arc_rad(plane, P1, P2, P3):
    if plane is None: return MAXINT

    x1, y1, z1 = P1
    x2, y2, z2 = P2
    x3, y3, z3 = P3

    if plane == 17: return rad1(x1,y1,x2,y2,x3,y3)
    if plane == 18: return rad1(x1,z1,x2,z2,x3,z3)
    if plane == 19: return rad1(y1,z1,y2,z2,y3,z3)
    return None, 0

def get_pts(plane, x,y,z):
    if plane == 17: return x,y
    if plane == 18: return x,z
    if plane == 19: return y,z

def one_quadrant(plane, c, p1, p2, p3):
    xc, yc = c
    x1, y1 = get_pts(plane, p1[0],p1[1],p1[2])
    x2, y2 = get_pts(plane, p2[0],p2[1],p2[2])
    x3, y3 = get_pts(plane, p3[0],p3[1],p3[2])

    def sign(x):
        if abs(x) < 1e-5: return 0
        if x < 0: return -1
        return 1

    signs = set((
        (sign(x1-xc),sign(y1-yc)),
        (sign(x2-xc),sign(y2-yc)),
        (sign(x3-xc),sign(y3-yc))
    ))

    if len(signs) == 1: return True

    if (1,1) in signs:
        signs.discard((1,0))
        signs.discard((0,1))
    if (1,-1) in signs:
        signs.discard((1,0))
        signs.discard((0,-1))
    if (-1,1) in signs:
        signs.discard((-1,0))
        signs.discard((0,1))
    if (-1,-1) in signs:
        signs.discard((-1,0))
        signs.discard((0,-1))

    if len(signs) == 1: return True

def arc_dir(plane, c, p1, p2, p3):
    xc, yc = c
    x1, y1 = get_pts(plane, p1[0],p1[1],p1[2])
    x2, y2 = get_pts(plane, p2[0],p2[1],p2[2])
    x3, y3 = get_pts(plane, p3[0],p3[1],p3[2])

    theta_start = atan2(y1-yc, x1-xc)
    theta_mid = atan2(y2-yc, x2-xc)
    theta_end = atan2(y3-yc, x3-xc)

    if theta_mid < theta_start:
        theta_mid = theta_mid + 2 * pi
    while theta_end < theta_mid:
        theta_end = theta_end + 2 * pi

    return theta_end < 2 * pi

def arc_fmt(plane, c1, c2, p1):
    x, y, z = p1
    if plane == 17: return "I%.4f J%.4f" % (c1-x, c2-y)
    if plane == 18: return "I%.4f K%.4f" % (c1-x, c2-z)
    if plane == 19: return "J%.4f K%.4f" % (c1-y, c2-z)


def douglas(st, tolerance=.001, plane=None, _first=True):
    """\
Perform Douglas-Peucker simplification on the path 'st' with the specified
tolerance.  The '_first' argument is for internal use only.

The Douglas-Peucker simplification algorithm finds a subset of the input points
whose path is never more than 'tolerance' away from the original input path.
If 'plane' is specified as 17, 18, or 19, it may find helical arcs in the given
plane in addition to lines.  Note that if there is movement in the plane
perpendicular to the arc, it will be distorted, so 'plane' should usually
be specified only when there is only movement on 2 axes
"""        
    if len(st) == 1:
        yield "G1", st[0], None
        return

    l1 = st[0]
    l2 = st[-1]

    worst_dist = 0
    worst = 0
    min_rad = MAXINT
    max_arc = -1

    ps = st[0]
    pe = st[-1]

    for i, p in enumerate(st):
        if p is l1 or p is l2: continue
        dist = dist_lseg(l1, l2, p)
        if dist > worst_dist:
            worst = i
            worst_dist = dist
            rad = arc_rad(plane, ps, p, pe)
            if rad < min_rad:
                max_arc = i
                min_rad = rad

    worst_arc_dist = 0
    if min_rad != MAXINT:
        c1, c2 = arc_center(plane, ps, st[max_arc], pe)
        lx, ly, lz = st[0]
        if one_quadrant(plane, (c1, c2), ps, st[max_arc], pe):
            for i, (x,y,z) in enumerate(st):
                if plane == 17: dist = abs(hypot(c1-x, c2-y) - min_rad)
                elif plane == 18: dist = abs(hypot(c1-x, c2-z) - min_rad)
                elif plane == 19: dist = abs(hypot(c1-y, c2-z) - min_rad)
                else: dist = MAXINT
                if dist > worst_arc_dist: worst_arc_dist = dist

                mx = (x+lx)/2
                my = (y+ly)/2
                mz = (z+lz)/2
                if plane == 17: dist = abs(hypot(c1-mx, c2-my) - min_rad)
                elif plane == 18: dist = abs(hypot(c1-mx, c2-mz) - min_rad)
                elif plane == 19: dist = abs(hypot(c1-my, c2-mz) - min_rad)
                else: dist = MAXINT
                lx, ly, lz = x, y, z
        else:
            worst_arc_dist = MAXINT
    else:
        worst_arc_dist = MAXINT

    if worst_arc_dist < tolerance and worst_arc_dist < worst_dist:
        ccw = arc_dir(plane, (c1, c2), ps, st[max_arc], pe)
        if plane == 18: ccw = not ccw
        yield "G1", ps, None
        if ccw:
            yield "G3", st[-1], arc_fmt(plane, c1, c2, ps)
        else:
            yield "G2", st[-1], arc_fmt(plane, c1, c2, ps)
    elif worst_dist > tolerance:
        if _first: yield "G1", st[0], None
        for i in douglas(st[:worst+1], tolerance, plane, False):
            yield i
        yield "G1", st[worst], None
        for i in douglas(st[worst:], tolerance, plane, False):
            yield i
        if _first: yield "G1", st[-1], None
    else:
        if _first: yield "G1", st[0], None
        if _first: yield "G1", st[-1], None

class Gcode:
    """\
    Create rs274ngc files compatible with GRBL    
    """    
    def __init__(self, homeheight = 1.5, safetyheight = 0.04,
                 tolerance=0.001, units="G20", header="",
                 preamble="", postamble="",
                 target=lambda s: sys.stdout.write(s + "\n"),
                 r_feed = 0,
                 c_feed = 0,
                 p_feed =0,
                 no_arcs = False,
                 grbl = False):
        self.lastx = self.lasty = self.lastz = self.lasta = None
        self.lastgcode = self.lastfeed = None
        self.homeheight = homeheight
        self.safetyheight = self.lastz = safetyheight
        self.tolerance = tolerance
        self.units = units
        self.cuts = []
        self.write = target
        self.time = 0
        self.plane = None
        self.header = header
        self.preamble = preamble
        self.postamble = postamble
        self.r_feed = r_feed
        self.c_feed = c_feed 
        self.p_feed = p_feed               
        self.no_arcs = no_arcs

    def comment(self,msg):
        self.write("( " + msg + " )")

    def set_plane(self, p):
        if (not self.no_arcs):
            assert p in (17,18,19)
            if p != self.plane:
                self.plane = p
                self.write("G%d" % p)

    def begin(self):
        """\
This function moves to the safety height, sets many modal codes to default
values, turns the spindle on at 3000RPM        
"""
        if self.header == "":
            self.write("(no header)")
        else:
            for line in self.header:
                self.write(line)

        if self.preamble == "":
            self.write("G17 G90 M3 S3000 G40 G94")
        else:
            self.write(self.preamble)    
    
        self.write(self.units)

        if not self.no_arcs:
            self.write("G91.1")

        self.write("G0 Z%.3f F%.3f" % (self.safetyheight,self.r_feed))

    def flush(self,feed=0,msg=""):
        """\
If any 'cut' moves are stored up, send them to the simplification algorithm
and actually output them.

This function is usually used internally (e.g., when changing from a cut
to a rapid) but can be called manually as well.  For instance, when
a contouring program reaches the end of a row, it may be desirable to enforce
that the last 'cut' coordinate is actually in the output file, and it may
give better performance because this means that the simplification algorithm
will examine fewer points per run.            
"""     
        if not self.cuts:
            return

        self.comment("BF " + msg)

        for move, (x, y, z), cent in douglas(self.cuts, self.tolerance, self.plane):
            if cent:
                self.write("%s X%.4f Y%.4f Z%.4f %s" % (move, x, y, z, cent))
                self.lastgcode = None
                self.lastx = x
                self.lasty = y
                self.lastz = z
            else:
                self.move_common("G1", x, y, z, feed = self.c_feed)
        
        self.comment("EF " + msg)        
        self.cuts = []

    def end(self):
        """
        Write postamble or "M2" if there is no postamble
        """
        #self.flush("End") # We do by hand setting the correct feed rate
        self.safety("END")
        if self.postamble=="":
            self.write("M2")
        else:
            self.write(self.postamble)


    def exactpath(self):
        """\
Set exact path mode.  Note that unless self.tolerance is set to zero,
the simplification algorithm may still skip over specified points.
"""        
        self.write("G61")

    # Set continuous mode.
    def continuous(self, tolerance=0.0):
    
        if tolerance > 0.0:
            self.write("G64 P%.4f" % tolerance)
        else:
            self.write("G64")

    def rapid(self, x=None, y=None, z=None, a=None,feed=None):
        #"Perform a rapid move to the specified coordinates"
        self.flush(feed,"rapid")
        self.move_common( "G0", x, y, z, a,feed)

    def move_common(self, gcode="G0",x=None, y=None, z=None, a=None, feed=None):
        #"An internal function used for G0 and G1 moves"
        gcodestring = xstring = ystring = zstring = astring = m_feed = ""

        if x == None: x = self.lastx

        if y == None: y = self.lasty

        if z == None: z = self.lastz

        if a == None: a = self.lasta

        if feed == None: feed = self.lastfeed 

        if x != self.lastx:
                xstring = " X%.4f" % (x)
                self.lastx = x

        if y != self.lasty:
                ystring = " Y%.4f" % (y)
                self.lasty = y

        if z != self.lastz:
                zstring = " Z%.4f" % (z)
                self.lastz = z

        if a != self.lasta:
                astring = " A%.4f" % (a)
                self.lasta = a

        if xstring == ystring == zstring == astring == "":
            return

        if gcode != self.lastgcode:
                gcodestring = gcode
                self.lastgcode = gcode
                 
        if feed != self.lastfeed:
            m_feed = " F%.3f" % feed
            self.lastfeed = feed
             
        cmd = "".join([gcodestring, xstring, ystring, zstring, astring, m_feed])
        if cmd:
            self.write(cmd)

    def set_feed(self, feed):
        #"Set the feed rate to the given value"
        # self.flush("Feed") # Maybe we have to eliminate this function
        self.write("F%.4f" % feed)

    def cut(self, x=None, y=None, z=None):
        #"Perform a cutting move at the specified feed rate to the specified coordinates"
        if self.cuts:
            lastx, lasty, lastz = self.cuts[-1]
        else:
            lastx, lasty, lastz = self.lastx, self.lasty, self.lastz
        if x is None: x = lastx
        if y is None: y = lasty
        if z is None: z = lastz
        self.cuts.append([x,y,z])

    def p_cut(self, x=None, y=None, z=None):
        print "Coord = ",x,y,z

    def home(self):
        #"Go to the 'home' height at rapid speed"
        # Maybe eliminate
        self.flush("Home")
        self.rapid(z=self.homeheight)

    def safety(self,msg=""):
        #"Go to the 'safety' height at rapid speed"
        #self.flush("Safety " + msg) # We flush by hand
        self.comment("S " + msg)
        self.rapid(z=self.safetyheight,feed=self.r_feed)


################################################################################
#             image-to-gcode                                                   #
#                                                                              #
################################################################################

epsilon = 1e-5

def ball_tool(r,rad):
    s = -sqrt(rad**2-r**2)
    return s

def endmill(r,dia, rough_offset=0.0):
    return 0

def vee_common(angle, rough_offset=0.0):
    slope = tan(pi/2.0 - (angle / 2.0) * pi / 180.0)
    def f(r, dia):
        return r * slope
    return f

def make_tool_shape(f, wdia, resp, rough_offset=0.0):
    # resp is pixel size
    res = 1. / resp
    wrad = wdia/2.0 + rough_offset
    rad = int(ceil((wrad-resp/2.0)*res))
    if rad < 1: rad = 1
    dia = 2*rad+1
    
    hdia = rad
    l = []
    for x in range(dia):
        for y in range(dia):
            r = hypot(x-hdia, y-hdia) * resp
            if r < wrad:
                z = f(r, wrad)
                l.append(z)
    #######################
    TOOL = Image_Matrix(dia,dia)
    l = []
    temp = []
    for x in range(dia):
        temp.append([])
        for y in range(dia):
            r = hypot(x-hdia, y-hdia) * resp
            if r < wrad:
                z = f(r, wrad)
                l.append(z)
                temp[x].append(float(z))
            else:
                temp[x].append(1e100000)
    TOOL.From_List(temp)
    TOOL.minus(TOOL.min()+rough_offset)
    return TOOL

def amax(seq):
    res = 0
    for i in seq:
        if abs(i) > abs(res): res = i
    return res

def group_by_sign(seq, slop=sin(pi/18), key=lambda x:x):
    sign = None
    subseq = []
    for i in seq:
        ki = key(i)
        if sign is None:
            subseq.append(i)
            if ki != 0:
                sign = ki / abs(ki)
        else:
            subseq.append(i)
            if sign * ki < -slop:
                sign = ki / abs(ki)
                yield subseq
                subseq = [i]
    if subseq: yield subseq

class Convert_Scan_Alternating:
    def __init__(self):
        self.st = 0

    def __call__(self, primary, items):
        st = self.st = self.st + 1
        if st % 2: items.reverse()
        if st == 1: yield True, items
        else: yield False, items

    def reset(self):
        self.st = 0

class Convert_Scan_Increasing:
    def __call__(self, primary, items):
        yield True, items

    def reset(self):
        pass

class Convert_Scan_Decreasing:
    def __call__(self, primary, items):
        items.reverse()
        yield True, items

    def reset(self):
        pass

class Convert_Scan_Upmill:
    def __init__(self, slop = sin(pi / 18)):
        self.slop = slop

    def __call__(self, primary, items):
        for span in group_by_sign(items, self.slop, operator.itemgetter(2)):
            if amax([it[2] for it in span]) < 0:
                span.reverse()
            yield True, span

    def reset(self):
        pass

class Convert_Scan_Downmill:
    def __init__(self, slop = sin(pi / 18)):
        self.slop = slop

    def __call__(self, primary, items):
        for span in group_by_sign(items, self.slop, operator.itemgetter(2)):
            if amax([it[2] for it in span]) > 0:
                span.reverse()
            yield True, span

    def reset(self):
        pass

class Reduce_Scan_Lace:
    def __init__(self, converter, slope, keep):
        self.converter = converter
        self.slope = slope
        self.keep = keep

    def __call__(self, primary, items):
        slope = self.slope
        keep = self.keep
        if primary:
            idx = 3
            test = operator.le
        else:
            idx = 2
            test = operator.ge

        def bos(j):
            return j - j % keep

        def eos(j):
            if j % keep == 0: return j
            return j + keep - j%keep

        for i, (flag, span) in enumerate(self.converter(primary, items)):
            subspan = []
            a = None
            for i, si in enumerate(span):
                ki = si[idx]
                if a is None:
                    if test(abs(ki), slope):
                        a = b = i
                else:
                    if test(abs(ki), slope):
                        b = i
                    else:
                        if i - b < keep: continue
                        yield True, span[bos(a):eos(b+1)]
                        a = None
            if a is not None:
                yield True, span[a:]

    def reset(self):
        self.converter.reset()

#############
class Reduce_Scan_Lace_new:
    def __init__(self, converter, depth, keep):
        self.converter = converter
        self.depth = depth
        self.keep = keep

    def __call__(self, primary, items):
        keep = self.keep
        max_z_cut = self.depth  # set a max z value to cut
        
        def bos(j):
            return j - j % keep

        def eos(j):
            if j % keep == 0: return j
            return j + keep - j%keep

        for i, (flag, span) in enumerate(self.converter(primary, items)):
            subspan = []
            a = None
            for i, si in enumerate(span):
                ki = si[1]         # This is (x,y,z)
                z_value   = ki[2]  # Get the z value from ki
                if a is None:
                    if z_value < max_z_cut:
                        a = b = i
                else:
                    if z_value < max_z_cut:
                        b = i
                    else:
                        if i - b < keep: continue
                        yield True, span[bos(a):eos(b+1)]
                        a = None
            if a is not None:
                yield True, span[a:]

    def reset(self):
        self.converter.reset()
#############
        
unitcodes = ['G20', 'G21']
convert_makers = [ Convert_Scan_Increasing, Convert_Scan_Decreasing, Convert_Scan_Alternating, Convert_Scan_Upmill, Convert_Scan_Downmill ]

def progress(a, b, START_TIME, GUI=[]):
    CUR_PCT = (a*100./b)
    if CUR_PCT > 100.0:
        CUR_PCT = 100.0
    MIN_REMAIN =( time()-START_TIME )/60 * (100-CUR_PCT)/CUR_PCT
    MIN_TOTAL = 100.0/CUR_PCT * ( time()-START_TIME )/60
    message = '%.1f %% ( %.1f Minutes Remaining | %.1f Minutes Total )' %( CUR_PCT, MIN_REMAIN, MIN_TOTAL )
    try:   
        GUI.statusMessage.set(message)
    except:
        fmessage(message)

class Converter:
    def __init__(self, BIG, 
                 image, units, tool_shape, pixel_size, pixelstep,
                 safetyheight, tolerance,feed,
                 convert_rows, convert_cols, cols_first_flag,
                 border, entry_cut, roughing_delta,rh_flag,
                 xoffset, yoffset, splitstep, header, 
                 preamble, postamble, edge_offset,
                 no_arcs,grbl):

        self.BIG = BIG  # Probably the Application window reference 
        self.image = image
        self.units = units
        self.tool_shape = tool_shape
        self.pixelsize = pixel_size
        self.safetyheight = safetyheight
        self.tolerance = tolerance
        self.convert_rows = convert_rows
        self.convert_cols = convert_cols
        self.cols_first_flag = cols_first_flag
        self.entry_cut = entry_cut
        self.rh_delta = roughing_delta
        self.rh_flag = rh_flag
        self.header = header
        self.preamble = preamble
        self.postamble = postamble
        self.border = border
        self.edge_offset = edge_offset
        self.no_arcs = no_arcs
        self.grbl_flag = grbl

        self.r_feed = feed[0]
        self.c_feed = feed[1]
        self.p_feed = feed[2]

        self.xoffset = xoffset
        self.yoffset = yoffset

        # Split step stuff
        splitpixels = 0
        if splitstep > epsilon:
            pixelstep   = int(floor(pixelstep * splitstep * 2))
            splitpixels = int(floor(pixelstep * splitstep    ))
        self.pixelstep   = pixelstep
        self.splitpixels = splitpixels

        self.cache = {}

        w, h = self.w, self.h = image.shape
        self.h1 = h
        self.w1 = w

        ### Percent complete stuff ###
        self.START_TIME=time()
        row_cnt=0
        cnt_border = 0
        if self.convert_rows != None:
            row_cnt = ceil( self.w1 / pixelstep) + 2
        col_cnt = 0
        if self.convert_cols != None:
            col_cnt = ceil( self.h1 / pixelstep) + 2
        if self.rh_delta != 0:
            cnt_mult = ceil(self.image.min() / -self.rh_delta) + 1
        else:
            cnt_mult = 1
        if self.convert_cols != None or self.convert_rows != None:
            cnt_border = 2
        self.cnt_total = (row_cnt + col_cnt + cnt_border )* cnt_mult
        print row_cnt,col_cnt
        self.cnt = 0.0

    def one_pass(self):
        g = self.g
        # g.set_feed(self.c_feed)
        g.comment("OP-S")
        if self.convert_cols and self.cols_first_flag:
            self.g.set_plane(19)
            self.mill_cols(self.convert_cols, True)
            if self.convert_rows: g.safety("OP-CC-CF")

        if self.convert_rows:
            self.g.set_plane(18)
            self.mill_rows(self.convert_rows, not self.cols_first_flag)

        if self.convert_cols and not self.cols_first_flag:
            self.g.set_plane(19)
            if self.convert_rows: g.safety("OP-CC-NFF")
            self.mill_cols(self.convert_cols, not self.convert_rows)

        g.safety("OPS")

        ## mill border ##
        if self.convert_cols:
            self.convert_cols.reset()
        if self.convert_rows:
            self.convert_rows.reset()

        step_save = self.pixelstep
        self.pixelstep = max(self.w1, self.h1) + 1
        if self.border == 1 and not self.convert_rows:
            if self.convert_cols:
                self.g.set_plane(18)
                self.mill_rows(self.convert_cols, True)
                g.safety("OP-B1-NCR")
                
        if self.border == 1 and not self.convert_cols:
            if self.convert_rows:
                self.g.set_plane(19)
                self.mill_cols(self.convert_rows, True)
                g.safety("OP-B1-NCC")
                
        self.pixelstep = step_save 

        if self.convert_cols:
            self.convert_cols.reset()
        if self.convert_rows:
            self.convert_rows.reset()
            
        g.safety("OPSE")

    def convert(self):
        output_gcode = []
            
        self.g = g = Gcode(safetyheight=self.safetyheight,
                           tolerance=self.tolerance,
                           units=self.units,
                           header=self.header,
                           preamble=self.preamble,
                           postamble=self.postamble, 
                           target=lambda s: output_gcode.append(s),
                           r_feed = self.r_feed,
                           c_feed = self.c_feed,
                           p_feed = self.p_feed,
                           no_arcs = self.no_arcs,
                           grbl = self.grbl_flag)

        g.begin()

        if not self.grbl_flag:
            g.continuous(self.tolerance)
        else:
            pass
            
        # g.safety("CF") # Not necessary we do the safety height in g.begin()
        
        if self.rh_flag:
            r = -self.rh_delta
            m = self.image.min()
            
            while r > m:
                self.rd = r
                self.one_pass()
                r = r - self.rh_delta
            if r < m + epsilon:
                self.rd = m
                self.one_pass()
        else:
            self.rd = self.image.min()
            self.one_pass()

        g.end()
        
        return output_gcode

    def get_z(self, x, y):
        try:
            return min(0, max(self.rd, self.cache[x,y]))
        except KeyError:
            self.cache[x,y] = d = self.image.height_calc(x,y,self.tool_shape)
            return min(0.0, max(self.rd, d))

    def get_dz_dy(self, x, y):
        y1 = max(0, y-1)
        y2 = min(self.image.shape[0]-1, y+1)
        dy = self.pixelsize * (y2-y1)
        return (self.get_z(x, y2) - self.get_z(x, y1)) / dy

    def get_dz_dx(self, x, y):
        x1 = max(0, x-1)
        x2 = min(self.image.shape[1]-1, x+1)
        dx = self.pixelsize * (x2-x1)
        return (self.get_z(x2, y) - self.get_z(x1, y)) / dx

    def frange(self,start, stop, step):
        out = []
        i = start
        while i < stop:
            out.append(i)
            i += step
        return out
            
    def mill_rows(self, convert_scan, primary):
        global STOP_CALC
        w1 = self.w1
        h1 = self.h1
        pixelsize = self.pixelsize
        pixelstep = self.pixelstep
        pixel_offset = int(ceil(self.edge_offset / pixelsize))
        jrange = self.frange(self.splitpixels+pixel_offset, w1-pixel_offset, pixelstep)
        if jrange[0] != pixel_offset: jrange.insert(0,pixel_offset)
        if w1-1-pixel_offset not in jrange: jrange.append(w1-1-pixel_offset)
        
        irange = range(pixel_offset,h1-pixel_offset)

        r_cnt = 0

        for j in jrange:
            self.cnt = self.cnt+1
            r_cnt = r_cnt + 1
            progress(self.cnt, self.cnt_total, self.START_TIME, self.BIG )
            y = (w1-j-1) * pixelsize + self.yoffset
            scan = []
            
            for i in irange:
                self.BIG.update()
                if STOP_CALC: return
                x = i * pixelsize + self.xoffset
                milldata = (i, (x, y, self.get_z(i, j)),
                            self.get_dz_dx(i, j), self.get_dz_dy(i, j))
                scan.append(milldata)

            for flag, points in convert_scan(primary, scan):
                if flag:
                    self.entry_cut(self, points[0][0], j, points)
                p_f = 0
                for p in points:
                    if p_f:
                        self.g.cut(*p[1])
                    else:
                        print "no p_f - %s" % (r_cnt)
                        self.g.move_common("G1", p[1][0],p[1][1],p[1][2], feed = self.p_feed)
                        p_f = 1
                        
            self.g.flush(feed = self.c_feed, msg = "- Row %s" % (r_cnt) )

    def mill_cols(self, convert_scan, primary):
        global STOP_CALC
        w1 = self.w1
        h1 = self.h1
        pixelsize = self.pixelsize
        pixelstep = self.pixelstep
        pixel_offset = int(ceil(self.edge_offset / pixelsize))
        jrange = self.frange(self.splitpixels+pixel_offset, h1-pixel_offset, pixelstep)
        if jrange[0] != pixel_offset: jrange.insert(0,pixel_offset)
        if h1-1-pixel_offset not in jrange: jrange.append(h1-1-pixel_offset)

        irange = range(pixel_offset,w1-pixel_offset)
        
        if h1-1-pixel_offset not in jrange: jrange.append(h1-1-pixel_offset)
        jrange.reverse()

        for j in jrange:
            self.cnt = self.cnt+1
            progress(self.cnt, self.cnt_total, self.START_TIME, self.BIG )
            x = j * pixelsize + self.xoffset
            scan = []
            for i in irange:
                self.BIG.update()
                if STOP_CALC: return
                y = (w1-i-1) * pixelsize + self.yoffset
                milldata = (i, (x, y, self.get_z(j, i)),
                            self.get_dz_dy(j, i), self.get_dz_dx(j, i))
                scan.append(milldata)
            for flag, points in convert_scan(primary, scan):
                if flag:
                    self.entry_cut(self, j, points[0][0], points)
                for p in points:
                    self.g.cut(*p[1])
            self.g.flush(feed = self.c_feed, msg = "Mill Col")

def generate(*args, **kw):
    """ 
    Is called from Apllication.WriteGCode around line 1020
    Call the Converter.convert() function around line 2900
    and return the Gcode generated
    
    """
    return Converter(*args, **kw).convert()

### Definition of the Entry_Cut types


class SimpleEntryCut:
    def __init__(self, feed):
        self.feed = feed

    def __call__(self, conv, i0, j0, points):
        p = points[0][1]
        conv.g.safety("SEC")
        conv.g.rapid(p[0], p[1],feed=self.feed)

# Calculate the portion of the arc to do so that none is above the
# safety height (that's just silly)
def circ(r,b):
    z = r**2 - (r-b)**2
    if z < 0: z = 0
    return z**.5

class ArcEntryCut:
    def __init__(self, feed, max_radius):
        self.feed = feed
        self.max_radius = max_radius

    def __call__(self, conv, i0, j0, points):
        if len(points) < 2:
            p = points[0][1]
            if self.feed:
                conv.g.set_feed(self.feed)
            conv.g.safety("AEC")
            conv.g.rapid(p[0], p[1])
            if self.feed:
                conv.g.set_feed(conv.feed)
            return

        p1 = points[0][1]
        p2 = points[1][1]
        z0 = p1[2]

        lim = int(ceil(self.max_radius / conv.pixelsize))
        r = range(1, lim)

        if self.feed:
            conv.g.set_feed(self.feed)
        conv.g.safety("AEC")

        x, y, z = p1

        pixelsize = conv.pixelsize

        cx = cmp(p1[0], p2[0])
        cy = cmp(p1[1], p2[1])

        radius = self.max_radius

        if cx != 0:
            h1 = conv.h1
            for di in r:
                dx = di * pixelsize
                i = i0 + cx * di
                if i < 0 or i >= h1: break
                z1 = conv.get_z(i, j0)
                dz = (z1 - z0)
                if dz <= 0: continue
                if dz > dx:
                    conv.g.write("(case 1)")
                    radius = dx
                    break
                rad1 = (dx * dx / dz + dz) / 2
                if rad1 < radius:
                    radius = rad1
                if dx > radius:
                    break

            z1 = min(p1[2] + radius, conv.safetyheight)

            x1 = p1[0] + cx * circ(radius, z1 - p1[2])
            conv.g.rapid(x1, p1[1])
            conv.g.cut(z=z1)

            I = - cx * circ(radius, z1 - p1[2])
            K = (p1[2] + radius) - z1
            
            conv.g.flush("AEC") # Broken for now 
            conv.g.lastgcode = None
            if cx > 0:
                #conv.g.write("G3 X%f Z%f R%f" % (p1[0], p1[2], radius)) #G3
                conv.g.write("G3 X%f Z%f I%f K%f" % (p1[0], p1[2], I, K))
            else:
                #conv.g.write("G2 X%f Z%f R%f" % (p1[0], p1[2], radius)) #G2
                conv.g.write("G2 X%f Z%f I%f K%f" % (p1[0], p1[2], I, K))
                
            conv.g.lastx = p1[0]
            conv.g.lasty = p1[1]
            conv.g.lastz = p1[2]
        else:
            w1 = conv.w1
            for dj in r:
                dy = dj * pixelsize
                j = j0 - cy * dj
                if j < 0 or j >= w1: break
                z1 = conv.get_z(i0, j)
                dz = (z1 - z0)
                if dz <= 0: continue
                if dz > dy:
                    radius = dy
                    break
                rad1 = (dy * dy / dz + dz) / 2
                if rad1 < radius: radius = rad1
                if dy > radius: break

            z1 = min(p1[2] + radius, conv.safetyheight)
            y1 = p1[1] + cy * circ(radius, z1 - p1[2])
            conv.g.rapid(p1[0], y1)
            conv.g.cut(z=z1)
            
            J =  -cy * circ(radius, z1 - p1[2])
            K = (p1[2] + radius) - z1
            
            conv.g.flush("AEC") # Broken for now
            conv.g.lastgcode = None
            
            if cy > 0:
                #conv.g.write("G2 Y%f Z%f R%f" % (p1[1], p1[2], radius)) #G2
                conv.g.write("G2 Y%f Z%f J%f K%f" % (p1[1], p1[2], J, K))
            else:
                #conv.g.write("G3 Y%f Z%f R%f" % (p1[1], p1[2], radius)) #G3
                conv.g.write("G3 Y%f Z%f J%f K%f" % (p1[1], p1[2], J, K))
            conv.g.lastx = p1[0]
            conv.g.lasty = p1[1]
            conv.g.lastz = p1[2]
        if self.feed:
            conv.g.set_feed(conv.feed)

class Image_Matrix:
    def __init__(self, width=2, height=2):
        self.width  = width
        self.height = height
        self.matrix = numarray.zeros((width, height), 'Float32')
        self.shape  = [width, height]
        self.t_offset = 0

    def __call__(self,i,j):
        return self.matrix[i+self.t_offset,j+self.t_offset]

    def Assign(self,i,j,val):
        fval=float(val)
        self.matrix[i+self.t_offset,j+self.t_offset]=fval

    def From_List(self,input_list):
        s = len(input_list)
        self.width  = s
        self.height = s

        self.matrix = numarray.zeros((s, s), 'Float32')
        for x in range(s):
            for y in range(s):
                self.matrix[x,y]=float(input_list[x][y])       

    def FromImage(self, im, pil_format):
        global STOP_CALC
        self.matrix = []

        if pil_format:
            him,wim = im.size
            self.matrix = numarray.zeros((wim, him), 'Float32')
            for i in range(0,wim):
                for j in range(0,him):
                    pix = im.getpixel((j,i))
                    self.matrix[i,j] = float(pix)
        else:
            him = im.width()
            wim = im.height()
            self.matrix = numarray.zeros((wim, him), 'Float32')
            for i in range(0,wim):
                for j in range(0,him):
                    try:    pix = im.get(j,i).split()
                    except: pix = im.get(j,i)
                    self.matrix[i,j] = float(pix[0])
                    
        self.width  = wim
        self.height = him
        self.shape  = [wim, him]
        self.t_offset = 0

    def pad_w_zeros(self,tool):
        ts = tool.width
        self.t_offset = (ts-1)/2 
        to = self.t_offset
        
        w, h = self.shape
        w1 = w + ts-1
        h1 = h + ts-1
        temp = numarray.zeros((w1, h1), 'Float32')
        for j in range(0, w1):
            for i in range(0, h1):
                temp[j,i] = -1e1000000
        temp[to:to+w, to:to+h] = self.matrix
        self.matrix = temp

    def height_calc(self,x,y,tool):
        to = self.t_offset
        ts = tool.width
        d= -1e100000
        m1 = self.matrix[y:y+ts, x:x+ts]
        d = (m1 - tool.matrix).max()
        return d

    def min(self):
        return self.matrix[self.t_offset:self.t_offset+self.width,
                              self.t_offset:self.t_offset+self.height].min()
    def max(self):
        return self.matrix[self.t_offset:self.t_offset+self.width,
                              self.t_offset:self.t_offset+self.height].max()
    def mult(self,val):
        self.matrix = self.matrix * float(val)
            
    def minus(self,val):
        self.matrix = self.matrix - float(val)
        
################################################################################
#             Function for outputting messages to different locations          #
#            depending on what options are enabled                             #
################################################################################
def fmessage(text,newline=True):
    global QUIET
    if not QUIET:
        if newline==True:
            try:
                sys.stdout.write(text)
                sys.stdout.write("\n")
            except:
                pass
        else:
            try:
                sys.stdout.write(text)
            except:
                pass

def message_box(title,message):
    if VERSION == 3:
        tkinter.messagebox.showinfo(title,message)
    else:
        tkMessageBox.showinfo(title,message)
        pass

def message_ask_ok_cancel(title, mess):
    if VERSION == 3:
        result=tkinter.messagebox.askokcancel(title, mess)
    else:
        result=tkMessageBox.askokcancel(title, mess)
    return result

################################################################################
#                          Startup Application                                 #
################################################################################
    
root = Tk()
app = Application(root)
app.master.title(p_name + " V " + version)
app.master.iconname(p_name)
app.master.minsize(960,540)

try: #Attempt to create temporary icon bitmap file
    f = open(p_name + "_icon",'w')
    f.write("#define " + p_name + "_icon_width 16\n")
    f.write("#define " + p_name + "_icon_height 16\n")
    f.write("static unsigned char " + p_name + "_icon_bits[] = {\n")
    f.write("   0x3f, 0xfc, 0x1f, 0xf8, 0xcf, 0xf3, 0x6f, 0xe4, 0x6f, 0xed, 0xcf, 0xe5,\n")
    f.write("   0x1f, 0xf4, 0xfb, 0xf3, 0x73, 0x98, 0x47, 0xce, 0x0f, 0xe0, 0x3f, 0xf8,\n")
    f.write("   0x7f, 0xfe, 0x3f, 0xfc, 0x9f, 0xf9, 0xcf, 0xf3 };\n")
    f.close()
    app.master.iconbitmap("@" + p_name + "_icon")
    os.remove(p_name + "_icon")
except:
    fmessage("Unable to create temporary icon file.")

root.mainloop()
