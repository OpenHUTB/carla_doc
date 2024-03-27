# 0.4.3-alpha (2019-08-29)

### Perfect

- **babel:** Upgrade babel compilation mode.

# 0.4.2-alpha (2019-08-29)

### Perfect

- **babel:** Upgrade babel compilation mode.

# 0.4.1-alpha (2019-08-29)

### Perfect

- **babel:** Upgrade babel compilation mode.

# 0.4.0-alpha (2019-06-27)

### Perfect

- **text:** Support for multiple lines of `text`.

# 0.3.7-alpha (2019-06-13)

### Bug Fixes

- **CRender:** Rendering level sorting exception (Attribute of index).

# 0.3.6-alpha (2019-06-12)

### Bug Fixes

- **Event:** `Click` callback is not responding.

# 0.3.5-alpha (2019-06-10)

### Bug Fixes

- **ES5:** Exception due to missing regenerator runtime module.

# 0.3.4-alpha (2019-06-06)

### Perfect

- **ES5:** Use babel to transcode to `ES5`.

# 0.3.3-alpha (2019-06-06)

### Bug Fixes

- **Style:** `opacity` is invalid for `gradientColor`.

# 0.3.2-alpha (2019-06-03)

### Bug Fixes

- **CRender:** An exception caused by missing import `deepClone`.

# 0.3.1-alpha (2019-06-02)

### Bug Fixes

- **Graph:** An exception caused by a negative radius (`circle`,`ellipse`,`arc`,`ring`,`sector`).

# 0.3.0-alpha (2019-06-01)

### Bug Fixes

- **Graph:** Fixed an exception for Graph.prototype.`attr`.

### CRender

- **prototype:** Add the CRender.prototype.`clone` to clone the Graph instance.

### Style

- **gradient:** Support `linear` and `radial` gradient.
- **prototype:** Add the Style.prototype.`getStyle` to get style configuration.


# 0.2.11-alpha (2019-05-29)

### Perfect

- **dependencies:** Change dependent plugin version.

# 0.2.10-alpha (2019-05-26)

### Perfect

- **core:** `animationDelay` Optimize animationDelay.

# 0.2.9-alpha (2019-05-24)

### Bug Fixes

- **graphs:** `extendNewGraph` Exception.

# 0.2.8-alpha (2019-05-21)

### Perfect

- **graph:** `draw` Remove unnecessary closepath (`polyline`,`smoothline`,`bezierCurve`).

# 0.2.7-alpha (2019-05-20)

### Bug Fixes

- **prototype:** `delGraph` Delete exception.

# 0.2.6-alpha (2019-05-14)

### Perfect

- **graph:** `polyline` Eliminate polyline blur due to 1px line width.

# 0.2.5-alpha (2019-05-12)

### Bug Fixes

- **prototype:** `delAllGraph` Incomplete deletion.

# 0.2.4-alpha (2019-05-12)

### Bug Fixes

- **prototype:** `delAllGraph` exception.

# 0.2.3-alpha (2019-04-29)

### Perfect

- **graph:** Improve graph basic properties.
- **Documentation:** Perfect documentation.

# 0.2.2-alpha (2019-04-25)

### Perfect

- **order:** Change the default render order of the graphs.(First Add First Render)
- **style:** Optimize lineDash rendering effects.

# 0.2.1-alpha (2019-04-24)

### Bug Fixes

- **CRender:** Initializing the canvas without setting its width and height attributes causes the rendering to be abnormal.

# 0.2.0-alpha (2019-04-15)

### Core

- **style:** Use the [Style](https://github.com/jiaming743/CRender#class-style) class to manage the graph style.
- **graph:** Instantiate graph using the [Graph](https://github.com/jiaming743/CRender#class-graph) class.
- **prototype:** Renamed some prototype methods.

### Features

- **graphs:** Add new graphs.([bezierCurve](https://github.com/jiaming743/CRender#bezierCurve),[text](https://github.com/jiaming743/CRender#text))
- **enhance:** `smoothline`, `bezierCurve`, `polyline` added the `closed` attribute.

# 0.1.4-alpha (2019-02-17)

### Bug Fixes

- **graphs:** `shape.length` is less than 3 draw exception.(`polyline`)
- **graphs:** Hover check exception when startAngle is more than endAngle.(`arc`,`sector`)

# 0.1.3-alpha (2019-02-16)

### Features

- **graphs:** Add new graphs ([polyline](https://github.com/jiaming743/CRender#polyline),[smoothline](https://github.com/jiaming743/CRender#smoothline))
- **prototype:** Add methos to delete graphs (`deleteAllElements`,`deleteElement`)