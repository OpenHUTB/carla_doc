[ENGLISH](./README_EN.md)

<h1 align="center">CRender</h1>

<p align="center">
    <a href="https://travis-ci.com/jiaming743/CRender"><img src="https://img.shields.io/travis/com/jiaming743/CRender.svg" alt="Travis CI"></a>
    <a href="https://github.com/jiaming743/CRender/blob/master/LICENSE"><img src="https://img.shields.io/github/license/jiaming743/CRender.svg" alt="LICENSE" /></a>
    <a href="https://www.npmjs.com/package/@jiaminghi/c-render"><img src="https://img.shields.io/npm/v/@jiaminghi/c-render.svg" alt="version" /></a>
</p>

### CRender是干什么的?

- 它是一个基于**canvas**的**矢量**图形渲染插件。
- 它对图形提供动画和鼠标事件支持。

### npm安装

```shell
$ npm install @jiaminghi/c-render
```

### 快速体验

```html
<!--资源位于个人服务器仅供体验和测试，请勿在生产环境使用-->
<!--调试版-->
<script src="http://lib.jiaminghi.com/crender/crender.map.js"></script>
<!--压缩版-->
<script src="http://lib.jiaminghi.com/crender/crender.min.js"></script>
<script>
  const { CRender, extendNewGraph } = window.CRender
  // do something
</script>
```

详细文档及示例请移步[HomePage](http://crender.jiaminghi.com).

- [使用](#使用)
- [Class CRender](#class-crender)
- [Class Graph](#class-graph)
- [Class Style](#class-style)
- [示例](#示例)
- [扩展新图形](#扩展新图形)
- [相关支持](#相关支持)

------

<h3 align="center">使用</h3>

```javascript
import CRender from '@jiaminghi/c-redner'

const canvas = document.getElementById('canvas')

// 实例化 CRender
const render = new CRender(canvas)

// 向render中添加图形
const circle = render.add({ name: 'circle', ... })
```

<h3 align="center">Class CRender</h3>

### 类

```javascript
/**
 * @description Class of CRender
 * @param {Object} canvas Canvas 节点
 * @return {CRender} CRender实例
 */
class CRender {
    // ...
}
```

### 实例属性

- [ctx](#ctx)
- [area](#area)
- [animationStatus](#animationStatus)
- [graphs](#graphs)
- [color](#color)
- [bezierCurve](#bezierCurve)

#### ctx

```javascript
/**
 * @description canvas context
 * @type {Object}
 * @example ctx = canvas.getContext('2d')
 */
```

#### area

```javascript
/**
 * @description canvas宽高
 * @type {Array<Number>}
 * @example area = [300，100]
 */
```

#### animationStatus

```javascript
/**
 * @description render是否处于动画渲染中
 * @type {Boolean}
 * @example animationStatus = true|false
 */
```

#### graphs

```javascript
/**
 * @description 已添加的图形
 * @type {Array<Graph>}
 * @example graphs = [Graph, Graph, ...]
 */
```

#### [color](https://github.com/jiaming743/color)

```javascript
/**
 * @description 颜色插件
 * @type {Object}
 */
```

#### [bezierCurve](https://github.com/jiaming743/BezierCurve)

```javascript
/**
 * @description 贝塞尔曲线插件
 * @type {Object}
 */
```

### 原型方法

- [add](#add)

  向render中添加图形

- [clone](#clone)

  克隆一个图形

- [delGraph](#delGraph)

  删除render中的一个图形

- [delAllGraph](#delAllGraph)

  删除render中所有的图形

- [drawAllGraph](#drawAllGraph)

  渲染render中所有的图形

- [clearArea](#clearArea)

  擦除canvas绘制区域

- [launchAnimation](#launchAnimation)

  使动画队列不为空且animationPause不为false的图形进行动画

#### add

```javascript
/**
 * @description 向render中添加图形
 * @param {Object} config 图形配置
 * @return {Graph} 图形实例
 */
CRender.prototype.add = function (config = {}) {
	// ...
}
```

#### Clone

```javascript
/**
 * @description 克隆一个图形
 * @param {Graph} graph 将要被克隆的图形
 * @return {Graph} 克隆的图形
 */
CRender.prototype.clone = function (graph) {
}
```

#### delGraph

```javascript
/**
 * @description 删除render中的一个图形
 * @param {Graph} graph 将要删除的图形实例
 * @return {Undefined} 无返回值
 */
CRender.prototype.delGraph = function (graph) {
	// ...
}
```

#### delAllGraph

```javascript
/**
 * @description 删除render中所有的图形
 * @return {Undefined} 无返回值
 */
CRender.prototype.delAllGraph = function () {
	// ...
}
```

#### drawAllGraph

```javascript
/**
 * @description 渲染render中所有的图形
 * @return {Undefined} 无返回值
 */
CRender.prototype.drawAllGraph = function () {
    // ...
}
```

#### clearArea

```javascript
/**
 * @description 擦除canvas绘制区域
 * @return {Undefined} 无返回值
 */
CRender.prototype.clearArea = function () {
	// ...
}
```

#### launchAnimation

```javascript
/**
 * @description 使动画队列不为空且animationPause不为false的图形进行动画
 * @return {Promise} Animation Promise
 */
CRender.prototype.launchAnimation = function () {
	// ...
}
```

<h3 align="center">Class Graph</h3>

### 实例属性

**当添加一个图形时，你可以配置这些属性。**

- [visible](#visible)
- [shape](#shape)
- [style](#style)
- [drag](#drag)
- [hover](#hover)
- [index](#index)
- [animationDelay](#animationDelay)
- [animationFrame](#animationFrame)
- [animationCurve](#animationCurve)
- [animationPause](#animationPause)
- [hoverRect](#hoverRect)
- [mouseEnter](#mouseEnter)
- [mouseOuter](#mouseOuter)
- [click](#click)

#### visible

```javascript
/**
 * @description 该图形是否可被渲染
 * @type {Boolean}
 * @default visible = true
 */
```

#### shape

```javascript
/**
 * @description 图形形状数据
 * @type {Object}
 */
```

#### [style](#Class-Style)

```javascript
/**
 * @description 图形样式数据 (Style实例)
 * @type {Style}
 */
```

#### drag

```javascript
/**
 * @description 是否启用拖拽功能
 * @type {Boolean}
 * @default drag = false
 */
```

#### hover

```javascript
/**
 * @description 是否启用悬浮检测
 * @type {Boolean}
 * @default hover = false
 */
```

#### index

```javascript
/**
 * @description 图形渲染层级，层级高者优先渲染
 * @type {Number}
 * @default index = 1
 */
```

#### animationDelay

```javascript
/**
 * @description 动画延迟时间(ms)
 * @type {Number}
 * @default animationDelay = 0
 */
```

#### animationFrame

```javascript
/**
 * @description 动画帧数
 * @type {Number}
 * @default animationFrame = 30
 */
```

#### [animationCurve](https://github.com/jiaming743/transition)

```javascript
/**
 * @description 动画缓动曲线
 * @type {String}
 * @default animationCurve = 'linear'
 */
```

#### animationPause

```javascript
/**
 * @description 是否暂停图形动画
 * @type {Boolean}
 * @default animationPause = false
 */
```

#### hoverRect

```javascript
/**
 * @description 矩形悬浮检测盒，配置该项则优先使用其进行鼠标悬浮检测
 * @type {Null|Array<Number>}
 * @default hoverRect = null
 * @example hoverRect = [0, 0, 100, 100] // [矩形起始点 x, y 坐标, 矩形宽, 高]
 */
```

#### mouseEnter

```javascript
/**
 * @description 鼠标进入图形事件处理器
 * @type {Null|Function}
 * @default mouseEnter = null
 */
```

#### mouseOuter

```javascript
/**
 * @description 鼠标移出图形事件处理器
 * @type {Null|Function}
 * @default mouseOuter = null
 */
```

#### click

```javascript
/**
 * @description 鼠标点击图形事件处理器
 * @type {Null|Function}
 * @default click = null
 */
```

#### Tip

启用图形的**mouseEnter**，**mouseOuter**，**click**等事件支持需要将`hover`属性配置为`true`。扩展的新图形需要配置**hoverCheck**方法以提供事件支持。

### 原型方法

- [attr](#attr)

  更新图形状态

- [animation](#animation)

  更新图形状态（伴随动画）

- [animationEnd](#animationEnd)

  跳至最后一帧动画

- [pauseAnimation](#pauseAnimation)

  暂停动画行为

- [playAnimation](#playAnimation)

  尝试动画行为

#### attr

```javascript
/**
 * @description 更新图形状态
 * @param {String} attrName 要更新的属性名
 * @param {Any} change      更新的值
 * @return {Undefined} 无返回值
 */
Graph.prototype.attr = function (attrName, change = undefined) {
	// ...
}
```

#### animation

```javascript
/**
 * @description 更新图形状态（伴随动画），仅支持shape和style属性
 * @param {String} attrName 要更新的属性名
 * @param {Any} change      更新的值
 * @param {Boolean} wait    是否存储动画队列，等待下次动画请求
 * @return {Promise} Animation Promise
 */
Graph.prototype.animation = async function (attrName, change, wait = false) {
	// ...
}
```

#### animationEnd

```javascript
/**
 * @description 跳至最后一帧动画
 * @return {Undefined} 无返回值
 */
Graph.prototype.animationEnd = function () {
  // ...
}
```

#### pauseAnimation

```javascript
/**
 * @description 暂停动画行为
 * @return {Undefined} 无返回值
 */
Graph.prototype.pauseAnimation = function () {
  // ...
}
```

#### playAnimation

```javascript
/**
 * @description 尝试动画行为
 * @return {Undefined} 无返回值
 */
Graph.prototype.playAnimation = function () {
  // ...
}
```

### 生命周期

当向**render**中添加图形时，你可以配置如下几个方法，它们将在特定时刻被调用。

- [added](#added)

  图形添加时被调用

- [beforeDraw](#beforeDraw)

  图形绘制前被调用

- [drawed](#drawed)

  图形绘制后被调用

- [beforeMove](#beforeMove)

  图形移动前被调用

- [moved](#moved)

  图形移动后被调用

- [beforeDelete](#beforeDelete)

  图形删除前被调用

- [deleted](#deleted)

  图形删除后被调用

#### added

```javascript
/**
 * @description 图形添加时被调用
 * @param {Graph} 图形实例
 */
config = {
  //...,
  added ({ shape, style }) {
    // 一些操作...
  }
}
```

#### beforeDraw

```javascript
/**
 * @description 图形绘制前被调用，图形样式已经初始化完毕
 *  你可以在此时修改ctx属性
 * @param {Graph} 图形实例
 * @param {CRender} CRender实例
 */
config = {
  //...,
  beforeDraw ({ shape, style }, { ctx }) {
    // 一些操作...
    ctx.stroke = 'transparent'
  }
}
```

#### drawed

```javascript
/**
 * @description 图形绘制后被调用
 * @param {Graph} 图形实例
 * @param {CRender} CRender实例
 */
config = {
  //...,
  drawed ({ shape, style }, { ctx }) {
    // 一些操作...
  }
}
```

#### beforeMove

```javascript
/**
 * @description 图形移动前被调用，移动行为发生前
 * @param {Event} 鼠标事件
 * @param {Graph} 图形实例
 */
config = {
  //...,
  beforeMove ({ offsetX, offsetY }, { shape, style }) {
    // 一些操作...
  }
}
```

#### moved

```javascript
/**
 * @description 图形移动后被调用，移动行为发生后
 * @param {Event} 鼠标事件
 * @param {Graph} 图形实例
 */
config = {
  //...,
  moved ({ offsetX, offsetY }, { shape, style }) {
    // 一些操作...
  }
}
```

#### beforeDelete

```javascript
/**
 * @description 图形删除前被调用
 * @param {Graph} 图形实例
 */
config = {
  //...,
  beforeDelete ({ shape, style }) {
    // 一些操作...
  }
}
```

#### deleted

```javascript
/**
 * @description 图形删除后被调用
 * @param {Graph} 图形实例
 */
config = {
  //...,
  deleted ({ shape, style }) {
    // 一些操作...
  }
}
```

<h3 align="center">Class Style</h3>

### 实例属性

- [fill](#fill)
- [stroke](#stroke)
- [opacity](#opacity)
- [lineCap](#lineCap)
- [lineJoin](#)
- [lineDash](#lineDash)
- [lineDashOffset](#lineDashOffset)
- [shadowBlur](#shadowBlur)
- [shadowColor](#shadowColor)
- [shadowOffsetX](#shadowOffsetX)
- [shadowOffsetY](#shadowOffsetY)
- [lineWidth](#lineWidth)
- [graphCenter](#graphCenter)
- [scale](#scale)
- [rotate](#rotate)
- [translate](#translate)
- [hoverCursor](#hoverCursor)
- [fontStyle](#fontStyle)
- [fontVarient](#fontVarient)
- [fontWeight](#fontWeight)
- [fontSize](#fontSize)
- [fontFamily](#fontFamily)
- [textAlign](#textAlign)
- [textBaseline](#textBaseline)
- [gradientColor](#gradientColor)
- [gradientType](#gradientType)
- [gradientParams](#gradientParams)
- [gradientWith](#gradientWith)
- [gradientStops](#gradientStops)
- [colors](#colors)

#### fill

```javascript
/**
 * @description 图形填充颜色的Rgba值
 * @type {Array<Number>}
 * @default fill = [0, 0, 0, 1]
 */
```

#### stroke

```javascript
/**
 * @description 图形描边颜色的Rgba值
 * @type {Array<Number>}
 * @default stroke = [0, 0, 0, 1]
 */
```

#### opacity

```javascript
/**
 * @description 图形透明度
 * @type {Number}
 * @default opacity = 1
 */
```

#### lineCap

```javascript
/**
 * @description Ctx的lineCap属性值
 * @type {Null|String}
 * @default lineCap = null
 * @example lineCap = 'butt'|'round'|'square'
 */
```

#### lineJoin

```javascript
/**
 * @description Ctx的lineJoin属性值
 * @type {Null|String}
 * @default lineJoin = null
 * @example lineJoin = 'round'|'bevel'|'miter'
 */
```

#### lineDash

```javascript
/**
 * @description Ctx的lineDash属性值
 * @type {Null|Array<Number>}
 * @default lineDash = null
 * @example lineDash = [10, 10]
 */
```

#### lineDashOffset

```javascript
/**
 * @description Ctx的lineDashOffset属性值
 * @type {Null|Number}
 * @default lineDashOffset = null
 * @example lineDashOffset = 10
 */
```

#### shadowBlur

```javascript
/**
 * @description Ctx的shadowBlur属性值
 * @type {Number}
 * @default shadowBlur = 0
 */
```

#### shadowColor

```javascript
/**
 * @description 图形阴影颜色的Rgba值
 * @type {Array<Number>}
 * @default shadowColor = [0, 0, 0, 0]
 */
```

#### shadowOffsetX

```javascript
/**
 * @description Ctx的shadowOffsetX属性值
 * @type {Number}
 * @default shadowOffsetX = 0
 */
```

#### shadowOffsetY

```javascript
/**
 * @description Ctx的shadowOffsetY属性值
 * @type {Number}
 * @default shadowOffsetY = 0
 */
```

#### lineWidth

```javascript
/**
 * @description Ctx的lineWidth属性值
 * @type {Number}
 * @default lineWidth = 0
 */
```

#### graphCenter

```javascript
/**
 * @description 图形中心点
 * @type {Null|Array<Number>}
 * @default graphCenter = null
 * @example graphCenter = [10, 10]
 */
```

#### scale

```javascript
/**
 * @description 图形缩放倍数
 * @type {Null|Array<Number>}
 * @default scale = null
 * @example scale = [1.5, 1.5]
 */
```

#### rotate

```javascript
/**
 * @description 图形旋转角度
 * @type {Null|Number}
 * @default rotate = null
 * @example rotate = 10
 */
```

#### translate

```javascript
/**
 * @description 图形位移距离
 * @type {Null|Array<Number>}
 * @default translate = null
 * @example translate = [10, 10]
 */
```

#### hoverCursor

```javascript
/**
 * @description 鼠标悬浮在图形上时cursor的值
 * @type {String}
 * @default hoverCursor = 'pointer'
 * @example hoverCursor = 'default'|'pointer'|'auto'|'crosshair'|'move'|'wait'|...
 */
```

#### fontStyle

```javascript
/**
 * @description Ctx的fontStyle属性值
 * @type {String}
 * @default fontStyle = 'normal'
 * @example fontStyle = 'normal'|'italic'|'oblique'
 */
```

#### fontVarient

```javascript
/**
 * @description Ctx的fontVarient属性值
 * @type {String}
 * @default fontVarient = 'normal'
 * @example fontVarient = 'normal'|'small-caps'
 */
```

#### fontWeight

```javascript
/**
 * @description Ctx的fontWeight属性值
 * @type {String|Number}
 * @default fontWeight = 'normal'
 * @example fontWeight = 'normal'|'bold'|'bolder'|'lighter'|Number
 */
```

#### fontSize

```javascript
/**
 * @description Ctx的fontSize属性值
 * @type {Number}
 * @default fontSize = 10
 */
```

#### fontFamily

```javascript
/**
 * @description Ctx的fontFamily属性值
 * @type {String}
 * @default fontFamily = 'Arial'
 */
```

#### textAlign

```javascript
/**
 * @description Ctx的textAlign属性值
 * @type {String}
 * @default textAlign = 'center'
 * @example textAlign = 'start'|'end'|'left'|'right'|'center'
 */
```

#### textBaseline

```javascript
/**
 * @description Ctx的textBaseline属性值
 * @type {String}
 * @default textBaseline = 'middle'
 * @example textBaseline = 'top'|'bottom'|'middle'|'alphabetic'|'hanging'
 */
```

#### gradientColor

```javascript
/**
 * @description 用于创建渐变色的颜色
 * @type {Null|Array<String>}
 * @default gradientColor = null
 * @example gradientColor = ['#000', '#111', '#222']
 */
```

#### gradientType

```javascript
/**
 * @description 渐变类型
 * @type {String}
 * @default gradientType = 'linear'
 * @example gradientType = 'linear' | 'radial'
 */
```

#### gradientParams

```javascript
/**
 * @description 渐变参数
 * @type {Array<Number>}
 * @default gradientParams = null
 * @example gradientParams = [x0, y0, x1, y1] (线性渐变)
 * @example gradientParams = [x0, y0, r0, x1, y1, r1] (径向渐变)
 */
```

#### gradientWith

```javascript
/**
 * @description 使用渐变色的属性
 * @type {String}
 * @default gradientWith = 'stroke'
 * @example gradientWith = 'stroke' | 'fill'
 */
```

#### gradientStops

```javascript
/**
 * @description 渐变色位置
 * @type {String|Array<Number>}
 * @default gradientStops = 'auto'
 * @example gradientStops = 'auto' | [0, .2, .3, 1]
 */
```

#### colors

```javascript
/**
 * @description 支持动画过渡的颜色容器
 * @type {Array<String>|Object}
 * @default colors = null
 * @example colors = ['#000', '#111', '#222']
 * @example colors = { a: '#000', b: '#111' }
 */
```

#### Tip

`gradientColor`和`gradientParams`被配置后将自动启用**渐变**。

### 原型方法

- [getStyle](#getStyle)

  获取图形当前样式配置

#### getStyle

```javascript
/**
 * @description 获取图形当前样式配置
 * @return {Object} 样式配置
 */
Style.prototype.getStyle = function () {
}
```

<h3 align="center">示例</h3>

CRender提供如下基础矢量图形。

- [圆形](#圆形)
- [椭圆形](#椭圆形)
- [矩形](#矩形)
- [环形](#环形)
- [弧形](#弧形)
- [扇形](#扇形)
- [正多边形](#正多边形)
- [折线](#折线)
- [折线（闭合）](#折线（闭合）)
- [光滑曲线](#光滑曲线)
- [光滑曲线（闭合）](#光滑曲线（闭合）)
- [贝塞尔曲线](#贝塞尔曲线)
- [贝塞尔曲线（闭合）](#贝塞尔曲线（闭合）)
- [文本](#文本)

#### 圆形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 圆心x轴坐标 |
| ry | `Number` | `0` | 圆心y轴坐标 |
| r | `Number` | `0` | 圆半径 |

```javascript
const { area: [w, h] } = render

const circleConfig = {
  name: 'circle',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    r: 50
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('shape', { r: 70 }, true)
    this.animation('style', { shadowBlur: 20 })
  },
  mouseOuter (e) {
    this.animation('shape', { r: 50 }, true)
    this.animation('style', { shadowBlur: 0 })
  }
}

const circle = render.add(circleConfig)
```

#### 椭圆形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 圆心x轴坐标 |
| ry | `Number` | `0` | 圆心y轴坐标 |
| hr | `Number` | `0` | 横轴半径 |
| vr | `Number` | `0` | 竖轴半径 |

```javascript
const { area: [w, h] } = render

const ellipseConfig = {
  name: 'ellipse',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    hr: 80,
    vr: 30
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    shadowColor: '#66eece',
    scale: [1, 1],
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { scale: [1.5, 1.5], shadowBlur: 20 })
  },
  mouseOuter (e) {
    this.animation('style', { scale: [1, 1], shadowBlur: 0 })
  }
}

const ellipse = render.add(ellipseConfig)
```

#### 矩形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| x | `Number` | `0` | 矩形左上角x轴坐标 |
| y | `Number` | `0` | 矩形左上角y轴坐标 |
| w | `Number` | `0` | 矩形宽度 |
| h | `Number` | `0` | 矩形高度 |

```javascript
const { area: [w, h] } = render

const rectConfig = {
  name: 'rect',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
  x: w / 2 - rectWidth / 2,
  y: h / 2 - rectHeight / 2,
  w: rectWidth,
  h: rectHeight
  },
  style: {
  fill: '#9ce5f4',
  shadowBlur: 0,
  shadowColor: '#66eece',
  hoverCursor: 'pointer',
  translate: [0, 0]
  },
  mouseEnter (e) {
    this.animation('shape', { w: 400 }, true)
    this.animation('style', { shadowBlur: 20, translate: [-100, 0] })
  },
  mouseOuter (e) {
    this.animation('shape', { w: 200 }, true)
    this.animation('style', { shadowBlur: 0, translate: [0, 0] })
  }
}

const rect = render.add(rectConfig)
```

#### 环形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 中心点x轴坐标 |
| ry | `Number` | `0` | 中心点y轴坐标 |
| r | `Number` | `0` | 环半径 |

```javascript
const { area: [w, h] } = render

const ringConfig = {
  name: 'ring',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    r: 50
  },
  style: {
    stroke: '#9ce5f4',
    lineWidth: 20,
    hoverCursor: 'pointer',
    shadowBlur: 0,
    shadowColor: '#66eece'
  },
  mouseEnter (e) {
    this.animation('style', { shadowBlur: 20, lineWidth: 30 })
  },
  mouseOuter (e) {
    this.animation('style', { shadowBlur: 0, lineWidth: 20 })
  }
}

const ring = render.add(ringConfig)
```

#### 弧形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 中心点x轴坐标 |
| ry | `Number` | `0` | 中心点y轴坐标 |
| r | `Number` | `0` | 弧半径 |
| startAngle | `Number` | `0` | 弧起始弧度值 |
| endAngle | `Number` | `0` | 弧结束弧度值 |
| clockWise | `Boolean` | `true` | 是否顺时针 |

```javascript
const { area: [w, h] } = render

const arcConfig = {
  name: 'arc',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    r: 60,
    startAngle: 0,
    endAngle: Math.PI / 3
  },
  style: {
    stroke: '#9ce5f4',
    lineWidth: 20,
    shadowBlur: 0,
    rotate: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('shape', { endAngle: Math.PI }, true)
    this.animation('style', { shadowBlur: 20, rotate: -30, lineWidth: 30 })
  },
  mouseOuter (e) {
    this.animation('shape', { endAngle: Math.PI / 3 }, true)
    this.animation('style', { shadowBlur: 0, rotate: 0, lineWidth: 20 })
  }
}

const arc = render.add(arcConfig)
```

#### 扇形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 中心点x轴坐标 |
| ry | `Number` | `0` | 中心点y轴坐标 |
| r | `Number` | `0` | 扇形半径 |
| startAngle | `Number` | `0` | 扇形起始弧度值 |
| endAngle | `Number` | `0` | 扇形结束弧度值 |
| clockWise | `Boolean` | `true` | 是否顺时针 |

```javascript
const { area: [w, h] } = render

const sectorConfig = {
  name: 'sector',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    r: 60,
    startAngle: 0,
    endAngle: Math.PI / 3
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    rotate: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('shape', { endAngle: Math.PI, r: 70 }, true)
    this.animation('style', { shadowBlur: 20, rotate: -30, lineWidth: 30 })
  },
  mouseOuter (e) {
    this.animation('shape', { endAngle: Math.PI / 3, r: 60 }, true)
    this.animation('style', { shadowBlur: 0, rotate: 0, lineWidth: 20 })
  }
}

const sector = render.add(sectorConfig)
```

#### 正多边形

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| rx | `Number` | `0` | 中心点x轴坐标 |
| ry | `Number` | `0` | 中心点y轴坐标 |
| r | `Number` | `0` | 外接圆半径 |
| side | `Number` | `0` | 边数 |

```javascript
const { area: [w, h] } = render

const regPolygonConfig = {
  name: 'regPolygon',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    rx: w / 2,
    ry: h / 2,
    r: 60,
    side: 6
  },
  style: {
    fill: '#9ce5f4',
    hoverCursor: 'pointer',
    shadowBlur: 0,
    rotate: 0,
    shadowColor: '#66eece'
  },
  mouseEnter (e) {
    this.animation('shape', { endAngle: Math.PI, r: 100 }, true)
    this.animation('style', { shadowBlur: 20, rotate: 180 })
  },
  mouseOuter (e) {
    this.animation('shape', { endAngle: Math.PI / 3, r: 60 }, true)
    this.animation('style', { shadowBlur: 0, rotate: 0 })
  }
}

const regPolygon = render.add(regPolygonConfig)
```

#### 折线

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| points | `Array` | `[]` | 构成折线的点 |
| close | `Boolean` | `false` | 是否闭合折线 |

```javascript
const { area: [w, h] } = render

const top = h / 3
const bottom = h / 3 * 2
const gap = w / 10

const beginX = w / 2 - gap * 2

const points = new Array(5).fill('').map((t, i) =>
  [beginX + gap * i, i % 2 === 0 ? top : bottom])

const polylineConfig = {
  name: 'polyline',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points
  },
  style: {
    stroke: '#9ce5f4',
    shadowBlur: 0,
    lineWidth: 10,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { lineWidth: 20, shadowBlur: 20 })
  },
  mouseOuter (e) {
    this.animation('style', { lineWidth: 10, shadowBlur: 0 })
  }
}

const polyline = render.add(polylineConfig)
```

#### 折线（闭合）

```javascript
const { area: [w, h] } = render

const top = h / 3
const bottom = h / 3 * 2
const gap = w / 10

const beginX = w / 2 - gap * 2

const points = new Array(5).fill('').map((t, i) =>
  [beginX + gap * i, i % 2 === 0 ? top : bottom])

points[2][1] += top * 1.3

const polylineClosedConfig = {
  name: 'polyline',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points,
    close: true
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    lineWidth: 10,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { shadowBlur: 20 }, true)
    const pointsCloned = deepClone(this.shape.points)
    pointsCloned[2][1] += top * 0.3
    this.animation('shape', { points: pointsCloned })
  },
  mouseOuter (e) {
    this.animation('style', { shadowBlur: 0 }, true)
    const pointsCloned = deepClone(this.shape.points)
    pointsCloned[2][1] -= top * 0.3
    this.animation('shape', { points: pointsCloned })
  }
}

const polylineClosed = render.add(polylineClosedConfig)
```

#### 光滑曲线

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| points | `Array` | `[]` | 构成光滑曲线的点 |
| close | `Boolean` | `false` | 是否闭合光滑曲线 |

```javascript
const { area: [w, h] } = render

const top = h / 3
const bottom = h / 3 * 2
const gap = w / 10

const beginX = w / 2 - gap * 2

const points = new Array(5).fill('').map((t, i) =>
  [beginX + gap * i, i % 2 === 0 ? top : bottom])

const smoothlineConfig = {
  name: 'smoothline',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points
  },
  style: {
    stroke: '#9ce5f4',
    shadowBlur: 0,
    lineWidth: 10,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { lineWidth: 20, shadowBlur: 20 })
  },
  mouseOuter (e) {
    this.animation('style', { lineWidth: 10, shadowBlur: 0 })
  }
}

const smoothline = render.add(smoothlineConfig)
```

#### 光滑曲线（闭合）

```javascript
import { getCircleRadianPoint } from '../../CRender/lib/util'

function getPoints (radius, centerPoint, pointNum) {
  const PIDived = Math.PI * 2 / pointNum

  const points = new Array(pointNum).fill('')
    .map((foo, i) =>
      getCircleRadianPoint(...centerPoint, radius, PIDived * i)
    )

  return points
}

const { area: [w, h] } = render

const radius = h / 3
const centerPoint = [w / 2, h / 2]

const smoothlineClosedConfig = {
  name: 'smoothline',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points: getPoints(radius, centerPoint, 3),
    close: true
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    lineWidth: 10,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { lineWidth: 20, shadowBlur: 20, rotate: 120 })
  },
  mouseOuter (e) {
    this.animation('style', { lineWidth: 10, shadowBlur: 0, rotate: 0 })
  },
  setGraphCenter (e, { style }) {
    if (e) {
      const { movementX, movementY } = e
      const [cx, cy] = style.graphCenter

      style.graphCenter = [cx + movementX, cy + movementY]
    } else {
      style.graphCenter = [...centerPoint]
    }
  }
}

const smoothlineClosed = render.add(smoothlineClosedConfig)
```

#### 贝塞尔曲线

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| points | `Array` | `[]` | 构成贝塞尔曲线的点 |
| close | `Boolean` | `false` | 是否闭合贝塞尔曲线 |

```javascript
const { area: [w, h] } = render

const offsetX = w / 2
const offsetY = h / 2

const points = [
  // 起始点
  [-100 + offsetX, -50 + offsetY],
  // 多段贝塞尔曲线
  [
    // 控制点1，控制点2，结束点
    [0  + offsetX, -50 + offsetY],
    [0  + offsetX, 50 + offsetY],
    [100  + offsetX, 50 + offsetY]
  ],
  // [...],[...]
]

const bezierCurveConfig = {
  name: 'bezierCurve',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points
  },
  style: {
    lineWidth: 10,
    stroke: '#9ce5f4',
    shadowBlur: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e) {
    this.animation('style', { lineWidth: 20, shadowBlur: 20 })
  },
  mouseOuter (e) {
    this.animation('style', { lineWidth: 10, shadowBlur: 0 })
  }
}

const bezierCurve = render.add(bezierCurveConfig)
```

#### 贝塞尔曲线（闭合）

```javascript
import { getCircleRadianPoint } from '../../CRender/lib/util'

function getPetalPoints (insideRadius, outsideRadius, petalNum, petalCenter) {
  const PI2Dived = Math.PI * 2 / (petalNum * 3)

  let points = new Array(petalNum * 3).fill('')
  .map((foo, i) => 
    getCircleRadianPoint(...petalCenter,
      i % 3 === 0 ? insideRadius : outsideRadius,
      PI2Dived * i)
  )

  const startPoint = points.shift()
  points.push(startPoint)

  points = new Array(petalNum).fill('')
  .map(foo => points.splice(0, 3))

  points.unshift(startPoint)

  return points
}

const { area: [w, h] } = render

const petalCenter = [w / 2, h / 2]
const [raidus1, raidus2, raidus3, raidus4] = [h / 6, h / 2.5, h / 3, h / 2]

const bezierCurveClosedConfig = {
  name: 'bezierCurve',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  shape: {
    points: getPetalPoints(raidus1, raidus2, 6, petalCenter),
    close: true
  },
  style: {
    fill: '#9ce5f4',
    shadowBlur: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer'
  },
  mouseEnter (e, { style: { graphCenter } }) {
    this.animation('style', { lineWidth: 20, shadowBlur: 20 }, true)
    this.animation('shape', { points: getPetalPoints(raidus3, raidus4, 6, graphCenter) })
  },
  mouseOuter (e, { style: { graphCenter } }) {
    this.animation('style', { lineWidth: 10, shadowBlur: 0 }, true)
    this.animation('shape', { points: getPetalPoints(raidus1, raidus2, 6, graphCenter) })
  },
  setGraphCenter (e, { style }) {
    if (e) {
      const { movementX, movementY } = e
      const [cx, cy] = style.graphCenter

      style.graphCenter = [cx + movementX, cy + movementY]
    } else {
      style.graphCenter = [...petalCenter]
    }
  }
}

const bezierCurveClosed = render.add(bezierCurveClosedConfig)
```

#### 文本

##### shape属性

| 属性名 | 类型 | 默认值 | 注解 |
| :---: | :---: | :---: | :---: |
| content | `String` | `''` | 文本内容 |
| position | `Array`  | `[0, 0]` | 文本起始位置 |
| maxWidth | `Number` | `Undefined` | 文本最大宽度 |
| rowGap | `Number` | `0` | 行间距 |

```javascript
const { area: [w, h] } = render

const centerPoint = [w / 2, h / 2]

const hoverRect = [w / 2 - 100, h / 2 - 30 ,200, 60]

const textConfig = {
  name: 'text',
  animationCurve: 'easeOutBack',
  hover: true,
  drag: true,
  hoverRect,
  shape: {
    content: 'CRender',
    position: centerPoint,
    maxWidth: 200
  },
  style: {
    fill: '#9ce5f4',
    fontSize: 50,
    shadowBlur: 0,
    rotate: 0,
    shadowColor: '#66eece',
    hoverCursor: 'pointer',
    scale: [1, 1],
    rotate: 0
  },
  mouseEnter (e) {
    this.animation('style', { shadowBlur: 20, scale: [1.5, 1.5], rotate: 30 })
  },
  mouseOuter (e) {
    this.animation('style', { shadowBlur: 0, scale: [1, 1], rotate: 0 })
  },
  moved (e, { hoverRect }) {
    const { movementX, movementY } = e

    hoverRect[0] += movementX
    hoverRect[1] += movementY
  }
}

const text = render.add(textConfig)
```

##### Tip

* 图形**text**的鼠标事件依赖`hoverRect`属性，如需鼠标事件生效请对其配置

* 文本中插入`\n`可以进行换行。

<h3 align="center">扩展新图形</h3>

CRender提供了一个方法去扩展新的图形，你可以**自定义**想要的图形。

```javascript
import { extendNewGraph } from '@jiaminghi/c-render'

const graphName = 'newGraph'
const graphConfig = {
  shape: { ... },
  // ...
}

extendNewGraph(graphName, graphConfig)
```

#### extendNewGraph

```javascript
/**
 * @description 扩展新图形
 * @param {String} name   图形名称
 * @param {Object} config 图形配置
 * @return {Undefined} 无返回值
 */
function extendNewGraph (name, config) {
    // ...
}
```

#### 图形配置属性

- [shape](#shape-required)
- [validator](#validator-required)
- [draw](#draw-required)
- [hoverCheck](#hoverCheck)
- [setGraphCenter](#setGraphCenter)
- [move](#move)
- [扩展示例](#扩展示例)

#### shape (必须)

```javascript
/**
 * @type {Object}
 * @description 图形形状数据
 */
config = {
  // ...,
  shape: {
    // 一些属性...
  }
}
```

#### validator (必须)

```javascript
/**
 * @type {Function}
 * @description 图形添加时将被调用，用于检测图形配置是否合法，
 *  若返回值为false则终止添加行为
 * @param {Graph} 当前图形实例
 * @return {Boolean} 配置是否合法
 */
config = {
  // ...,
  validator ({ shape }) {
    // 检查图形配置...
    // return true | false
  }
}
```

#### draw (必须)

```javascript
/**
 * @type {Function}
 * @description 图形绘制器
 * @param {CRender} 当前CRender实例
 * @param {Graph}   当前图形实例
 * @return {Undefined} 无返回值
 */
config = {
  // ...,
  draw ({ ctx }, { shape }) {
    // 绘制...
  }
}
```

#### hoverCheck (可选)

```javascript
/**
 * @type {Function}
 * @description 通过鼠标位置去判断当前图形是否处于鼠标悬浮状态，
 *  用于给mouseEnter, mouseOuter, drag, click事件提供支持。
 * @param {Array<Number>} 鼠标位置
 * @param {Graph}         当前图形实例
 * @return {Boolean} 是否处于鼠标悬浮状态
 */
config = {
  // ...,
  validator ([offsetX, offsetY], { shape }) {
    // 检测是否处于鼠标悬浮状态...
    // return true | false
  }
}
```

#### setGraphCenter (可选)

```javascript
/**
 * @type {Function}
 * @description 设置图形中心点
 *  提供rotate, scale and translate支持
 *  添加图形及图形被拖动后将被调用
 * @param {Event} 鼠标事件 (图形被添加时调用，该参数为null)
 * @param {Graph} 当前图形实例
 * @return {Undefined} 无返回值
 */
config = {
  // ...,
  setGraphCenter ([offsetX, offsetY], { style }) {
    // style.graphCenter = [offsetX, offsetY]
  }
}
```

#### move (Optional)

```javascript
/**
 * @type {Function}
 * @description Moving graph,support for drag
 * @param {Event} Mouse move Event
 * @param {Graph} Current graph instance
 * @return {Undefined} Void
 */
config = {
  // ...,
  move ([offsetX, offsetY], { shape }) {
    // 一些操作...
  }
}
```

#### 扩展示例

```javascript
import { extendNewGraph } from '@jiaminghi/c-render'

const circle = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0
  },

  validator ({ shape }) {
    const { rx, ry, r } = shape

    if (typeof rx !== 'number' || typeof ry !== 'number' || typeof r !== 'number') {
      console.error('Shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    const { rx, ry, r } = shape

    ctx.arc(rx, ry, r, 0, Math.PI * 2)

    ctx.fill()
    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape }) {
    const { rx, ry, r } = shape

    return checkPointIsInCircle(rx, ry, r, position)
  },

  setGraphCenter (e, { shape, style }) {
    const { rx, ry } = shape

    style.graphCenter = [rx, ry]
  },

  move ({ movementX, movementY }, { shape }) {
    this.attr('shape', {
      rx: shape.rx + movementX,
      ry: shape.ry + movementY
    })
  }
}

extendNewGraph('circle', circle)
```

<h3 align="center">相关支持</h3>

- [Transition](https://github.com/jiaming743/transition)

  提供动画过渡数据 (animationCurve)。

  如果你想扩展新的缓动曲线，请移步 [扩展新曲线](http://transition.jiaminghi.com/guide/#扩展缓动曲线).

- [BezierCurve](https://github.com/jiaming743/bezierCurve)

  提供贝塞尔曲线支持，例如计算曲线长度，曲线折线互转。

- [Color](https://github.com/jiaming743/Color)

  提供了颜色计算，例如获取颜色的rgba值，以便于颜色动画状态的计算。