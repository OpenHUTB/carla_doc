import beziercurve from '@jiaminghi/bezier-curve'

import {
  deepClone,
  eliminateBlur,
  checkPointIsInCircle,
  getTwoPointDistance,
  checkPointIsInSector,
  getRegularPolygonPoints,
  checkPointIsInPolygon,
  checkPointIsNearPolyline,
  checkPointIsInRect
} from '../plugin/util'

import {
  drawPolylinePath,
  drawBezierCurvePath
} from '../plugin/canvas'

const { polylineToBezierCurve, bezierCurveToPolyline } = beziercurve

export const circle = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0
  },

  validator ({ shape }) {
    const { rx, ry, r } = shape

    if (typeof rx !== 'number' || typeof ry !== 'number' || typeof r !== 'number') {
      console.error('Circle shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    const { rx, ry, r } = shape

    ctx.arc(rx, ry, r > 0 ? r : 0.01, 0, Math.PI * 2)

    ctx.fill()
    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape }) {
    const { rx, ry, r } = shape

    return checkPointIsInCircle(position, rx, ry, r)
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

export const ellipse = {
  shape: {
    rx: 0,
    ry: 0,
    hr: 0,
    vr: 0
  },

  validator ({ shape }) {
    const { rx, ry, hr, vr } = shape

    if (typeof rx !== 'number' || typeof ry !== 'number' || typeof hr !== 'number' || typeof vr !== 'number') {
      console.error('Ellipse shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    let { rx, ry, hr, vr } = shape

    ctx.ellipse(rx, ry, hr > 0 ? hr : 0.01, vr > 0 ? vr : 0.01, 0, 0, Math.PI * 2)

    ctx.fill()
    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape }) {
    const { rx, ry, hr, vr } = shape

    const a = Math.max(hr, vr)
    const b = Math.min(hr, vr)

    const c = Math.sqrt(a * a - b * b)

    const leftFocusPoint = [rx - c, ry]
    const rightFocusPoint = [rx + c, ry]

    const distance = getTwoPointDistance(position, leftFocusPoint) + getTwoPointDistance(position, rightFocusPoint)

    return distance <= 2 * a
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

export const rect = {
  shape: {
    x: 0,
    y: 0,
    w: 0,
    h: 0
  },

  validator ({ shape }) {
    const { x, y, w, h } = shape

    if (typeof x !== 'number' || typeof y !== 'number' || typeof w !== 'number' || typeof h !== 'number') {
      console.error('Rect shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    let { x, y, w, h } = shape

    ctx.rect(x, y, w, h)

    ctx.fill()
    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape }) {
    let { x, y, w, h } = shape

    return checkPointIsInRect(position, x, y, w, h)
  },

  setGraphCenter (e, { shape, style }) {
    const { x, y, w, h } = shape

    style.graphCenter = [x + w / 2, y + h / 2]
  },

  move ({ movementX, movementY }, { shape }) {
    this.attr('shape', {
      x: shape.x + movementX,
      y: shape.y + movementY
    })
  }
}

export const ring = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0
  },

  validator ({ shape }) {
    const { rx, ry, r } = shape

    if (typeof rx !== 'number' || typeof ry !== 'number' || typeof r !== 'number') {
      console.error('Ring shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    const { rx, ry, r } = shape

    ctx.arc(rx, ry, r > 0 ? r : 0.01, 0, Math.PI * 2)

    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape, style }) {
    const { rx, ry, r } = shape

    const { lineWidth } = style

    const halfLineWidth = lineWidth / 2

    const minDistance = r - halfLineWidth
    const maxDistance = r + halfLineWidth

    const distance = getTwoPointDistance(position, [rx, ry])

    return (distance >= minDistance && distance <= maxDistance)
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

export const arc = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0,
    startAngle: 0,
    endAngle: 0,
    clockWise: true
  },

  validator ({ shape }) {
    const keys = ['rx', 'ry', 'r', 'startAngle', 'endAngle']

    if (keys.find(key => typeof shape[key] !== 'number')) {
      console.error('Arc shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    const { rx, ry, r, startAngle, endAngle, clockWise } = shape

    ctx.arc(rx, ry, r > 0 ? r : 0.001, startAngle, endAngle, !clockWise)

    ctx.stroke()

    ctx.closePath()
  },

  hoverCheck (position, { shape, style }) {
    const { rx, ry, r, startAngle, endAngle, clockWise } = shape

    const { lineWidth } = style

    const halfLineWidth = lineWidth / 2

    const insideRadius = r - halfLineWidth
    const outsideRadius = r + halfLineWidth

    return !checkPointIsInSector(position, rx, ry, insideRadius, startAngle, endAngle, clockWise) &&
    checkPointIsInSector(position, rx, ry, outsideRadius, startAngle, endAngle, clockWise)
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

export const sector = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0,
    startAngle: 0,
    endAngle: 0,
    clockWise: true
  },

  validator ({ shape }) {
    const keys = ['rx', 'ry', 'r', 'startAngle', 'endAngle']

    if (keys.find(key => typeof shape[key] !== 'number')) {
      console.error('Sector shape configuration is abnormal!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    ctx.beginPath()

    const { rx, ry, r, startAngle, endAngle, clockWise } = shape

    ctx.arc(rx, ry, r > 0 ? r : 0.01, startAngle, endAngle, !clockWise)

    ctx.lineTo(rx, ry)

    ctx.closePath()

    ctx.stroke()
    ctx.fill()
  },

  hoverCheck (position, { shape }) {
    const { rx, ry, r, startAngle, endAngle, clockWise } = shape

    return checkPointIsInSector(position, rx, ry, r, startAngle, endAngle, clockWise)
  },

  setGraphCenter (e, { shape, style }) {
    const { rx, ry } = shape

    style.graphCenter = [rx, ry]
  },

  move ({ movementX, movementY }, { shape }) {
    const { rx, ry } = shape

    this.attr('shape', {
      rx: rx + movementX,
      ry: ry + movementY
    })
  }
}

export const regPolygon = {
  shape: {
    rx: 0,
    ry: 0,
    r: 0,
    side: 0
  },

  validator ({ shape }) {
    const { side } = shape

    const keys = ['rx', 'ry', 'r', 'side']

    if (keys.find(key => typeof shape[key] !== 'number')) {
      console.error('RegPolygon shape configuration is abnormal!')

      return false
    }

    if (side < 3) {
      console.error('RegPolygon at least trigon!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape, cache }) {
    ctx.beginPath()

    const { rx, ry, r, side } = shape

    if (!cache.points || cache.rx !== rx || cache.ry !== ry || cache.r !== r || cache.side !== side) {
      const points = getRegularPolygonPoints(rx, ry, r, side)

      Object.assign(cache, { points, rx, ry, r, side })
    }

    const { points } = cache

    drawPolylinePath(ctx, points)

    ctx.closePath()

    ctx.stroke()
    ctx.fill()
  },

  hoverCheck (position, { cache }) {
    let { points } = cache

    return checkPointIsInPolygon(position, points)
  },

  setGraphCenter (e, { shape, style }) {
    const { rx, ry } = shape

    style.graphCenter = [rx, ry]
  },

  move ({ movementX, movementY }, { shape, cache }) {
    const { rx, ry } = shape

    cache.rx += movementX
    cache.ry += movementY

    this.attr('shape', {
      rx: rx + movementX,
      ry: ry + movementY
    })

    cache.points = cache.points.map(([x, y]) => [x + movementX, y + movementY])
  }
}

export const polyline = {
  shape: {
    points: [],
    close: false
  },

  validator ({ shape }) {
    const { points } = shape

    if (!(points instanceof Array)) {
      console.error('Polyline points should be an array!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape, style: { lineWidth } }) {
    ctx.beginPath()

    let { points, close } = shape

    if (lineWidth === 1) points = eliminateBlur(points)

    drawPolylinePath(ctx, points)

    if (close) {
      ctx.closePath()

      ctx.fill()
      ctx.stroke()
    } else {
      ctx.stroke()
    }
  },

  hoverCheck (position, { shape, style }) {
    const { points, close } = shape

    const { lineWidth } = style

    if (close) {
      return checkPointIsInPolygon(position, points)
    } else {
      return checkPointIsNearPolyline(position, points, lineWidth)
    }
  },

  setGraphCenter (e, { shape, style }) {
    const { points } = shape

    style.graphCenter = points[0]
  },

  move ({ movementX, movementY }, { shape }) {
    const { points } = shape

    const moveAfterPoints = points.map(([x, y]) => [x + movementX, y + movementY])

    this.attr('shape', {
      points: moveAfterPoints
    })
  }
}

export const smoothline = {
  shape: {
    points: [],
    close: false
  },

  validator ({ shape }) {
    const { points } = shape

    if (!(points instanceof Array)) {
      console.error('Smoothline points should be an array!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape, cache }) {
    const { points, close } = shape

    if (!cache.points || cache.points.toString() !== points.toString()) {
      const bezierCurve = polylineToBezierCurve(points, close)
      const hoverPoints = bezierCurveToPolyline(bezierCurve)

      Object.assign(cache, {
        points: deepClone(points, true),
        bezierCurve,
        hoverPoints
      })
    }

    const { bezierCurve } = cache

    ctx.beginPath()

    drawBezierCurvePath(ctx, bezierCurve.slice(1), bezierCurve[0])

    if (close) {
      ctx.closePath()

      ctx.fill()
      ctx.stroke()
    } else {
      ctx.stroke()
    }
  },

  hoverCheck (position, { cache, shape, style }) {
    const { hoverPoints } = cache

    const { close } = shape

    const { lineWidth } = style

    if (close) {
      return checkPointIsInPolygon(position, hoverPoints)
    } else {
      return checkPointIsNearPolyline(position, hoverPoints, lineWidth)
    }
  },

  setGraphCenter (e, { shape, style }) {
    const { points } = shape

    style.graphCenter = points[0]
  },

  move ({ movementX, movementY }, { shape, cache }) {
    const { points } = shape

    const moveAfterPoints = points.map(([x, y]) => [x + movementX, y + movementY])

    cache.points = moveAfterPoints

    const [fx, fy] = cache.bezierCurve[0]
    const curves = cache.bezierCurve.slice(1)

    cache.bezierCurve = [
      [fx + movementX, fy + movementY],
      ...curves.map(curve => curve.map(([x, y]) => [x + movementX, y + movementY]))
    ]

    cache.hoverPoints = cache.hoverPoints.map(([x, y]) => [x + movementX, y + movementY])

    this.attr('shape', {
      points: moveAfterPoints
    })
  }
}

export const bezierCurve = {
  shape: {
    points: [],
    close: false
  },

  validator ({ shape }) {
    const { points } = shape

    if (!(points instanceof Array)) {
      console.error('BezierCurve points should be an array!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape, cache }) {
    let { points, close } = shape

    if (!cache.points || cache.points.toString() !== points.toString()) {
      const hoverPoints = bezierCurveToPolyline(points, 20)

      Object.assign(cache, {
        points: deepClone(points, true),
        hoverPoints
      })
    }

    ctx.beginPath()

    drawBezierCurvePath(ctx, points.slice(1), points[0])

    if (close) {
      ctx.closePath()

      ctx.fill()
      ctx.stroke()
    } else {
      ctx.stroke()
    }
  },

  hoverCheck (position, { cache, shape, style }) {
    const { hoverPoints } = cache

    const { close } = shape

    const { lineWidth } = style

    if (close) {
      return checkPointIsInPolygon(position, hoverPoints)
    } else {
      return checkPointIsNearPolyline(position, hoverPoints, lineWidth)
    }
  },

  setGraphCenter (e, { shape, style }) {
    const { points } = shape

    style.graphCenter = points[0]
  },

  move ({ movementX, movementY }, { shape, cache }) {
    const { points } = shape

    const [fx, fy] = points[0]
    const curves = points.slice(1)

    const bezierCurve = [
      [fx + movementX, fy + movementY],
      ...curves.map(curve => curve.map(([x, y]) => [x + movementX, y + movementY]))
    ]

    cache.points = bezierCurve
    cache.hoverPoints = cache.hoverPoints.map(([x, y]) => [x + movementX, y + movementY])

    this.attr('shape', {
      points: bezierCurve
    })
  }
}

export const text = {
  shape: {
    content: '',
    position: [],
    maxWidth: undefined,
    rowGap: 0
  },

  validator ({ shape }) {
    const { content, position, rowGap } = shape

    if (typeof content !== 'string') {
      console.error('Text content should be a string!')

      return false
    }

    if (!(position instanceof Array)) {
      console.error('Text position should be an array!')

      return false
    }

    if (typeof rowGap !== 'number') {
      console.error('Text rowGap should be a number!')

      return false
    }

    return true
  },

  draw ({ ctx }, { shape }) {
    let { content, position, maxWidth, rowGap } = shape

    const { textBaseline, font } = ctx

    const fontSize = parseInt(font.replace(/\D/g, ''))

    let [x, y] = position

    content = content.split('\n')
    const rowNum = content.length

    const lineHeight = fontSize + rowGap
    const allHeight = rowNum * lineHeight - rowGap

    let offset = 0

    if (textBaseline === 'middle') {
      offset = allHeight / 2
      y += fontSize / 2
    }

    if (textBaseline === 'bottom') {
      offset = allHeight
      y += fontSize
    }

    position = new Array(rowNum).fill(0).map((foo, i) => [x, y + i * lineHeight - offset])

    ctx.beginPath()

    content.forEach((text, i) => {
      ctx.fillText(text, ...position[i], maxWidth)
      ctx.strokeText(text, ...position[i], maxWidth)
    })

    ctx.closePath()
  },

  hoverCheck (position, { shape, style }) {
    return false
  },

  setGraphCenter (e, { shape, style }) {
    const { position } = shape

    style.graphCenter = [...position]
  },

  move ({ movementX, movementY }, { shape }) {
    const { position: [x, y] } = shape

    this.attr('shape', {
      position: [x + movementX, y + movementY]
    })
  }
}

const graphs = new Map([
  ['circle', circle],
  ['ellipse', ellipse],
  ['rect', rect],
  ['ring', ring],
  ['arc', arc],
  ['sector', sector],
  ['regPolygon', regPolygon],
  ['polyline', polyline],
  ['smoothline', smoothline],
  ['bezierCurve', bezierCurve],
  ['text', text]
])

export default graphs

/**
 * @description Extend new graph
 * @param {String} name   Name of Graph
 * @param {Object} config Configuration of Graph
 * @return {Undefined} Void
 */
export function extendNewGraph (name, config) {
  if (!name || !config) {
    console.error('ExtendNewGraph Missing Parameters!')

    return
  }

  if (!config.shape) {
    console.error('Required attribute of shape to extendNewGraph!')

    return
  }

  if (!config.validator) {
    console.error('Required function of validator to extendNewGraph!')

    return
  }

  if (!config.draw) {
    console.error('Required function of draw to extendNewGraph!')

    return
  }

  graphs.set(name, config)
}
