[ENGLISH](./README_EN.md)

<h1 align="center">Color Extension</h1>

<p align="center">
  <a href="https://travis-ci.com/DataV-Team/color">
    <img src="https://img.shields.io/travis/com/DataV-Team/color.svg" alt="Travis CI" />
  </a>
  <a href="https://github.com/DataV-Team/color/blob/master/LICENSE">
    <img src="https://img.shields.io/github/license/DataV-Team/color.svg" alt="LICENSE" />
  </a>
  <a href="https://www.npmjs.com/package/@jiaminghi/color">
    <img src="https://img.shields.io/npm/v/@jiaminghi/color.svg" alt="NPM" />
  </a>
</p>

### 这是一个提供如下几个方法的颜色插件。

- **[isHex](#isHex)**

  判断一个颜色是否为 Hex 类型

- **[isRgb](#isRgb)**

  判断一个颜色是否为 Rgb 类型

- **[isRgba](#isRgba)**

  判断一个颜色是否为 Rgba 类型

- **[isRgbOrRgba](#isRgbOrRgba)**

  判断一个颜色是否为 Rgb 或 Rgba 类型

- **[darken](#darken)**

  加深颜色

- **[lighten](#lighten)**

  提亮颜色

- **[fade](#fade)**

  调节颜色透明度

- **[toHex](#toHex)**

  转换成 Hex 颜色

- **[toRgb](#toRgb)**

  将颜色转为 Rgb 或 Rgba 颜色

- **[getOpacity](#getOpacity)**

  获取颜色透明度

- **[getRgbValue](#getRgbValue)**

  获取颜色的 Rgb 值

- **[getRgbaValue](#getRgbaValue)**

  获取颜色的 Rgba 值

- **[getColorFromRgbValue](#getColorFromRgbValue)**

  从 Rgb 或 Rgba 值获取颜色

### 附录

- **[颜色关键字](#颜色关键字)**

  内置颜色关键字

### npm 安装

```shell
$ npm install @jiaminghi/color
```

### 使用

```javascript
import { toHex } from '@jiaminghi/color'

// do something
```

### 快速体验

```html
<!--调试版-->
<script src="https://unpkg.com/@jiaminghi/color/dist/index.js"></script>
<!--压缩版-->
<script src="https://unpkg.com/@jiaminghi/color/dist/index.min.js"></script>
<script>
  const { darken, lighten } = window.Color
  // do something
</script>
```

---

<h3 align="center">示例</h3>

#### isHex

```typescript
/**
 * @description 判断一个颜色是否为hex类型
 * @param {string} color 可能是hex颜色的字符串
 * @return {boolean} 判断结果
 */
type isHex = (color: string) => boolean

isHex('#000') // true
isHex('#333333') // true
isHex('Not A Color') // false
isHex('rgb(0,0,0)') // false
isHex('rgba(0,0,0,1)') // false
```

#### isRgb

```typescript
/**
 * @description 判断一个颜色是否为rgb类型
 * @param {string} color 可能是rgb颜色的字符串
 * @return {boolean} 判断结果
 */
type isRgb = (color: string) => boolean

isRgb('rgb(0,0,0)') // true
isRgb('RGB(0,0,0)') // true
isRgb('Not A Color') // false
isRgb('#000') // false
isRgb('#000000') // false
```

#### isRgba

```typescript
/**
 * @description 判断一个颜色是否为rgba类型
 * @param {string} color 可能是rgba颜色的字符串
 * @return {boolean} 判断结果
 */
type isRgba = (color: string) => boolean

isRgba('rgba(0,0,0,1)') // true
isRgba('rgb(0,0,0)') // false
isRgba('Not A Color') // false
isRgba('#000') // false
isRgba('#000000') // false
```

#### isRgbOrRgba

```typescript
/**
 * @description 判断一个颜色是否为rgb或rgba类型
 * @param {string} color 可能是rgb或rgba颜色的字符串
 * @return {boolean} 判断结果
 */
type isRgbOrRgba = (color: string) => boolean

isRgbOrRgba('rgb(0,0,0)') // true
isRgbOrRgba('RGB(0,0,0)') // true
isRgbOrRgba('rgba(0,0,0,1)') // true
isRgbOrRgba('#000') // false
isRgbOrRgba('Not A Color') // false
```

#### darken

```typescript
/**
 * @description 加深颜色
 * @param {string} color   Hex|Rgb|Rgba颜色或颜色关键字
 * @param {number} percent 加深的百分比 (1-100)
 * @return {string} Rgba颜色 (无效输入将抛出异常)
 */
type darken = (color: string, percent: number) => string

const before = '#3080E8'

const after = darken(color, 20)
// after = 'rgba(0,77,181,1)'
```

<p align="center">
  <img width="180px" src="./exampleImgs/1.jpg" />
</p>

#### lighten

```typescript
/**
 * @description 提亮颜色
 * @param {string} color   Hex|Rgb|Rgba颜色或颜色关键字
 * @param {number} percent 提亮的百分比 (1-100)
 * @return {string} Rgba颜色 (无效输入将抛出异常)
 */
type lighten = (color: string, percent: number) => string

const before = '#3080E8'

const after = lighten(color, 20)
// after = 'rgba(99,179,255,1)'
```

<p align="center">
    <img width="180px" src="./exampleImgs/2.jpg" />
</p>

#### fade

```typescript
/**
 * @description 调节颜色透明度
 * @param {string} color   Hex|Rgb|Rgba颜色或颜色关键字
 * @param {number} Percent 透明度百分比
 * @return {string} Rgba颜色 (无效输入将抛出异常)
 */
type fade = (color: string, percent: number) => string

const before = '#3080E8'

const after = lighten(color, 20)
// after = 'rgba(48,128,232,0.2)'
```

<p align="center">
  <img width="180px" src="./exampleImgs/3.jpg" />
</p>

#### toHex

```typescript
/**
 * @description 转换成Hex颜色
 * @param {string} color Hex|Rgb|Rgba颜色或颜色关键字
 * @return {string} Hex颜色 (无效输入将抛出异常)
 */
type toHex = (color: string) => string

const before = 'rgb(48,128,232)'

const after = toHex(before)
// after = '#3080e8'
```

#### toRgb

```typescript
/**
 * @description 将颜色转为Rgb或Rgba颜色
 * @param {string} color   Hex|Rgb|Rgba颜色或颜色关键字
 * @param {number} opacity 颜色的透明度 (输入该参数将生成Rgba颜色)
 * @return {string} Rgb或Rgba颜色 (无效输入将抛出异常)
 */
type toRgb = (color: string, opacity: number) => string

const before = '#3080E8'

const after1 = toRgb(before)
// after1 = 'rgb(48,128,232)'
const after2 = toRgb(before, 0.2)
// after2 = 'rgba(48,128,232,0.2)'
```

#### getOpacity

```typescript
/**
 * @description 获取颜色透明度
 * @param {string} color Hex|Rgb|Rgba颜色或颜色关键字
 * @return {number} 颜色透明度 (无效输入将抛出异常)
 */
type getOpacity = (color: string) => number

const color1 = '#3080E8'
const color2 = 'rgba(48,128,232,0.2)'

const opacity1 = getOpacity(color1)
// opacity1 = 1
const opacity2 = getOpacity(color2)
// opacity2 = 0.2
```

#### getRgbValue

```typescript
/**
 * @description 获取颜色的Rgb值
 * @param {string} color Hex|Rgb|Rgba颜色或颜色关键字
 * @return {RgbValue} Rgb值 (无效输入将抛出异常)
 */
type RgbValue = [number, number, number]
type getRgbValue = (color: string) => RgbValue

const color = '#3080E8'

const rgbValue = getRgbValue(color)
// rgbValue = [48, 128, 232]
```

#### getRgbaValue

```typescript
/**
 * @description 获取颜色的Rgba值
 * @param {String} color Hex|Rgb|Rgba颜色或颜色关键字
 * @return {RgbaValue|null} Rgba值 (无效输入将抛出异常)
 */
type RgbaValue = [number, number, number, number]
type getRgbaValue = (color: string) => RgbaValue

const color1 = '#3080E8'
const color2 = 'rgba(48,128,232,0.2)'

const rgbaValue1 = getRgbaValue(color1)
// rgbaValue1 = [48, 128, 232, 1]
const rgbaValue2 = getRgbaValue(color2)
// rgbaValue2 = [48, 128, 232, 0.2]
```

#### getColorFromRgbValue

```typescript
/**
 * @description 从Rgb或Rgba值获取颜色
 * @param {Value} value Rgb或Rgba颜色的值
 * @return {string} Rgb颜色或Rgba颜色 (无效输入将抛出异常)
 */
type RgbValue = [number, number, number]
type RgbaValue = [number, number, number, number]
type Value = RgbValue | RgbaValue
type getColorFromRgbValue = (value: Value) => string

const value1 = [48, 128, 232]
const value2 = [48, 128, 232, 0.2]

const color1 = getColorFromRgbValue(value1)
// color1 = 'rgb(48,128,232)'
const color2 = getColorFromRgbValue(value2)
// color2 = 'rgba(48,128,232,0.2)'
```

#### 颜色关键字

<p align="center">
  <img width="750px" src="./exampleImgs/keywords.png" />
</p>
