[ENGLISH](./README_EN.md)

<h1 align="center">Transition</h1>

<p align="center">
    <a href="https://travis-ci.com/jiaming743/transition"><img src="https://img.shields.io/travis/com/jiaming743/transition.svg" alt="Travis CI"></a>
    <a href="https://github.com/jiaming743/transition/blob/master/LICENSE"><img src="https://img.shields.io/github/license/jiaming743/transition.svg" alt="LICENSE" /> </a>
    <a href="https://www.npmjs.com/package/@jiaminghi/transition"><img src="https://img.shields.io/npm/v/@jiaminghi/transition.svg" alt="NPM" /></a>
</p>

### Transition是什么?

- 它是一个基于**贝塞尔曲线**的动效插件。
- 它提供常用的**缓动曲线**。
- 支持**自定义**缓动曲线。

### 动画是如何产生的?

* 获取一帧动画数据
* 根据动画数据渲染图像
* 重复...

我们可以使用三组数据去描述一段动画（**动画起始状态**、**动画结束状态**、**缓动曲线**），根据这三组数据我们可以计算出动画过程中每一帧动画的状态。这就是***Transition***所提供的功能，根据每一帧动画的状态，我们不断的进行重绘，动画就产生了。

### npm安装

```shell
$ npm install @jiaminghi/transition
```

### 使用

```javascript
import { transition, injectNewCurve } from '@jiaminghi/transition'

// do something
```

### 快速体验

```html
<!--资源位于个人服务器仅供体验和测试，请勿在生产环境使用-->
<!--调试版-->
<script src="http://lib.jiaminghi.com/transition/transition.map.js"></script>
<!--压缩版-->
<script src="http://lib.jiaminghi.com/transition/transition.min.js"></script>
<script>
  const { transition, injectNewCurve } = window.transition
  // do something
</script>
```

详细文档及示例请移步[HomePage](http://transition.jiaminghi.com).

- [注解](#注解)
- [示例](#示例)
- [扩展新曲线](#扩展新曲线)
- [缓动曲线表](#缓动曲线表)

------

<h3 align="center">注解</h3>

```javascript
/**
 * @description 根据动画起止状态及缓动曲线获取若干帧动画状态数据
 * @param {String|Array} tBC               缓动曲线名称或曲线数据
 * @param {Number|Arrya|Object} startState 动画起始状态
 * @param {Number|Arrya|Object} endState   动画结束状态
 * @param {Number} frameNum                动画帧数
 * @param {Boolean} deep                   是否启用递归模式
 * @return {Array} 每一帧的动画数据
 */
function transition (tBC, startState = null, endState = null, frameNum = 30, deep = false) {  // ...
}
```

<h3 align="center">示例</h3>

**Transition** 支持三种数据类型以描述动画状态.

* [Number](#Number)
* [Array](#Array)
* [Object](#Annotation)
* [Recursive](#Recursive)

#### Number

```javascript
import transition from '@jiaminghi/transition'

const beginState = 0
const endState = 100

const animationState = transition('linear', beginState, endState, 10)

/**
 * animationState = [
 *   0, 11.03429355281208, 22.126200274348417, 33.259259259259245, 44.41700960219478,
 *   55.58299039780521, 66.74074074074073, 77.87379972565157, 88.96570644718793, 100
 * ]
 * /
```

#### Array

```javascript
import transition from '@jiaminghi/transition'

const beginState = [10, 20, 30]
const endState = [100, 200, 300]

const animationState = transition('linear', beginState, endState, 10)

/**
 * animationState = [
 *   [10, 20, 30],
 *   [32.415625, 64.83125, 97.24687499999999],
 *   [55, 110, 165],
 *   [77.58437500000001, 155.16875000000002, 232.753125],
 *   [100, 200, 300]
 * ]
 * /
```

#### Object

```javascript
import transition from '@jiaminghi/transition'

const objectBeginState = { x: 10, y: 10, r: 5}
const objectEndState = { x: 100, y: 10, r: 5}

const animationState = transition('linear', objectBeginState, objectEndState, 5)

/**
 * animationState = [
 *   {x: 10, y: 10, r: 5},
 *   {x: 32.415625, y: 10, r: 5},
 *   {x: 55, y: 10, r: 5},
 *   {x: 77.58437500000001, y: 10, r: 5},
 *   {x: 100, y: 10, r: 5}
 * ]
 * /
```

#### Recursive

启用递归模式以计算`Array`或`Object`中的深层数据.

```javascript
import transition from '@jiaminghi/transition'

const beginState = {
  points: [ [10, 30], [20, 80] ],
  origin: { x: 10, y: 20 },
  radius: 3
}

const endState = {
  points: [ [100, 230], [120, 10] ],
  origin: { x: 100, y: 200 },
  radius: 9
}

const animationState = transition('linear', beginState, endState, 3, true)

/**
 * animationState = [
 *   {
 *     origin: { x: 10, y: 20 },
 *     points: [ [10, 30], [20, 80] ],
 *     radius: 3
 *   },
 *   {
 *     origin: { x: 55, y: 110 },
 *     points: [ [55, 130], [70, 45] ],
 *     radius: 6
 *   },
 *   {
 *     origin: { x: 100, y: 200 },
 *     points: [ [100, 230], [120, 10] ],
 *     radius: 9
 *   }
 * ]
 * /
```

**Notice**

* 非数值的属性或元素不参与计算过程.
* 起始状态与结束状态的数据类型（包括属性及元素的数量）必须保持一致.

<h3 align="center">扩展新曲线</h3>

如果你想扩展新的缓动曲线，你可以使用`Transition`提供的`injectNewCurve`方法去扩展。

```javascript
import { injectNewCurve } from '@jiaminghi/transition'

const curveName = 'linear'

// 可以使用绘制工具获得
const bezierCurve = [[[0, 1]],[[1, 0]]]

injectNewCurve(curveName, bezierCurve)
```

[缓动曲线绘制工具](http://transition.jiaminghi.com/draw/)

<h3 align="center">缓动曲线表</h3>

* [linear](#linear)
* [easeInSine](#easeInSine)
* [easeOutSine](#easeOutSine)
* [easeInOutSine](#easeInOutSine)
* [easeInQuad](#easeInQuad)
* [easeOutQuad](#easeOutQuad)
* [easeInOutQuad](#easeInOutQuad)
* [easeInCubic](#easeInCubic)
* [easeOutCubic](#easeOutCubic)
* [easeInOutCubic](#easeInOutCubic)
* [easeInQuart](#easeInQuart)
* [easeOutQuart](#easeOutQuart)
* [easeInOutQuart](#easeInOutQuart)
* [easeInQuint](#easeInQuint)
* [easeOutQuint](#easeOutQuint)
* [easeInOutQuint](#easeInOutQuint)
* [easeInBack](#easeInBack)
* [easeOutBack](#easeOutBack)
* [easeInOutBack](#easeInOutBack)
* [easeInElastic](#easeInElastic)
* [easeOutElastic](#easeOutElastic)
* [easeInOutElastic](#easeInOutElastic)
* [easeInBounce](#easeInBounce)
* [easeOutBounce](#easeOutBounce)
* [easeInOutBounce](#easeInOutBounce)

#### linear
![linear](/exampleImg/linear.gif)

#### easeInSine
![linear](/exampleImg/easeInSine.gif)

#### easeOutSine
![linear](/exampleImg/easeOutSine.gif)

#### easeInOutSine
![linear](/exampleImg/easeInOutSine.gif)

#### easeInQuad
![linear](/exampleImg/easeInQuad.gif)

#### easeOutQuad
![linear](/exampleImg/easeOutQuad.gif)

#### easeInOutQuad
![linear](/exampleImg/easeInOutQuad.gif)

#### easeInCubic
![linear](/exampleImg/easeInCubic.gif)

#### easeOutCubic
![linear](/exampleImg/easeOutCubic.gif)

#### easeInOutCubic
![linear](/exampleImg/easeInOutCubic.gif)

#### easeInQuart
![linear](/exampleImg/easeInQuart.gif)

#### easeOutQuart
![linear](/exampleImg/easeOutQuart.gif)

#### easeInOutQuart
![linear](/exampleImg/easeInOutQuart.gif)

#### easeInQuint
![linear](/exampleImg/easeInQuint.gif)

#### easeOutQuint
![linear](/exampleImg/easeOutQuint.gif)

#### easeInOutQuint
![linear](/exampleImg/easeInOutQuint.gif)

#### easeInBack
![linear](/exampleImg/easeInBack.gif)

#### easeOutBack
![linear](/exampleImg/easeOutBack.gif)

#### easeInOutBack
![linear](/exampleImg/easeInOutBack.gif)

#### easeInElastic
![linear](/exampleImg/easeInElastic.gif)

#### easeOutElastic
![linear](/exampleImg/easeOutElastic.gif)

#### easeInOutElastic
![linear](/exampleImg/easeInOutElastic.gif)

#### easeInBounce
![linear](/exampleImg/easeInBounce.gif)

#### easeOutBounce
![linear](/exampleImg/easeOutBounce.gif)

#### easeInOutBounce
![linear](/exampleImg/easeInOutBounce.gif)