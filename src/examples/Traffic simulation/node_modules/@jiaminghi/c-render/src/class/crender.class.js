import color from '@jiaminghi/color'
import bezierCurve from '@jiaminghi/bezier-curve'

import { deepClone } from '../plugin/util'

import allGraph from '../config/graphs'

import Graph from './graph.class'

/**
 * @description           Class of CRender
 * @param {Object} canvas Canvas DOM
 * @return {CRender}      Instance of CRender
 */
export default class CRender {
  constructor (canvas) {
    if (!canvas) {
      console.error('CRender Missing parameters!')

      return
    }

    const ctx = canvas.getContext('2d')

    const { clientWidth, clientHeight } = canvas

    const area = [clientWidth, clientHeight]

    canvas.setAttribute('width', clientWidth)
    canvas.setAttribute('height', clientHeight)

    /**
     * @description Context of the canvas
     * @type {Object}
     * @example ctx = canvas.getContext('2d')
     */
    this.ctx = ctx
    /**
     * @description Width and height of the canvas
     * @type {Array}
     * @example area = [300，100]
     */
    this.area = area
    /**
     * @description Whether render is in animation rendering
     * @type {Boolean}
     * @example animationStatus = true|false
     */
    this.animationStatus = false
    /**
     * @description Added graph
     * @type {[Graph]}
     * @example graphs = [Graph, Graph, ...]
     */
    this.graphs = []
    /**
     * @description Color plugin
     * @type {Object}
     * @link https://github.com/jiaming743/color
     */
    this.color = color
    /**
     * @description Bezier Curve plugin
     * @type {Object}
     * @link https://github.com/jiaming743/BezierCurve
     */
    this.bezierCurve = bezierCurve

    // bind event handler
    canvas.addEventListener('mousedown', mouseDown.bind(this))
    canvas.addEventListener('mousemove', mouseMove.bind(this))
    canvas.addEventListener('mouseup', mouseUp.bind(this))
  }
}

/**
 * @description        Clear canvas drawing area
 * @return {Undefined} Void
 */
CRender.prototype.clearArea = function () {
  const { area } = this

  this.ctx.clearRect(0, 0, ...area)
}

/**
 * @description           Add graph to render
 * @param {Object} config Graph configuration
 * @return {Graph}        Graph instance
 */
CRender.prototype.add = function (config = {}) {
  const { name } = config

  if (!name) {
    console.error('add Missing parameters!')

    return
  }

  const graphConfig = allGraph.get(name)

  if (!graphConfig) {
    console.warn('No corresponding graph configuration found!')

    return
  }

  const graph = new Graph(graphConfig, config)

  if (!graph.validator(graph)) return

  graph.render = this

  this.graphs.push(graph)

  this.sortGraphsByIndex()

  this.drawAllGraph()

  return graph
}

/**
 * @description Sort the graph by index
 * @return {Undefined} Void
 */
CRender.prototype.sortGraphsByIndex = function () {
  const { graphs } = this

  graphs.sort((a, b) => {
    if (a.index > b.index) return 1
    if (a.index === b.index) return 0
    if (a.index < b.index) return -1
  })
}

/**
 * @description         Delete graph in render
 * @param {Graph} graph The graph to be deleted
 * @return {Undefined}  Void
 */
CRender.prototype.delGraph = function (graph) {
  if (typeof graph.delProcessor !== 'function') return

  graph.delProcessor(this)

  this.graphs = this.graphs.filter(graph => graph)

  this.drawAllGraph()
}

/**
 * @description        Delete all graph in render
 * @return {Undefined} Void
 */
CRender.prototype.delAllGraph = function () {
  this.graphs.forEach(graph => graph.delProcessor(this))

  this.graphs = this.graphs.filter(graph => graph)

  this.drawAllGraph()
}

/**
 * @description        Draw all the graphs in the render
 * @return {Undefined} Void
 */
CRender.prototype.drawAllGraph = function () {
  this.clearArea()

  this.graphs.filter(graph => graph && graph.visible).forEach(graph => graph.drawProcessor(this, graph))
}

/**
 * @description      Animate the graph whose animation queue is not empty
 *                   and the animationPause is equal to false
 * @return {Promise} Animation Promise
 */
CRender.prototype.launchAnimation = function () {
  const { animationStatus } = this

  if (animationStatus) return

  this.animationStatus = true

  return new Promise(resolve => {
    animation.call(this, () => {
      this.animationStatus = false

      resolve()
    }, Date.now())
  })
}

/**
 * @description Try to animate every graph
 * @param {Function} callback Callback in animation end
 * @param {Number} timeStamp  Time stamp of animation start
 * @return {Undefined} Void
 */
function animation (callback, timeStamp) {
  const { graphs } = this

  if (!animationAble(graphs)) {
    callback()

    return
  }

  graphs.forEach(graph => graph.turnNextAnimationFrame(timeStamp))

  this.drawAllGraph()

  requestAnimationFrame(animation.bind(this, callback, timeStamp))
}

/**
 * @description Find if there are graph that can be animated
 * @param {[Graph]} graphs
 * @return {Boolean}
 */
function animationAble (graphs) {
  return graphs.find(graph => !graph.animationPause && graph.animationFrameState.length)
}

/**
 * @description Handler of CRender mousedown event
 * @return {Undefined} Void
 */
function mouseDown (e) {
  const { graphs } = this

  const hoverGraph = graphs.find(graph => graph.status === 'hover')

  if (!hoverGraph) return

  hoverGraph.status = 'active'
}

/**
 * @description Handler of CRender mousemove event
 * @return {Undefined} Void
 */
function mouseMove (e) {
  const { offsetX, offsetY } = e
  const position = [offsetX, offsetY]

  const { graphs } = this

  const activeGraph = graphs.find(graph => (graph.status === 'active' || graph.status === 'drag'))

  if (activeGraph) {
    if (!activeGraph.drag) return

    if (typeof activeGraph.move !== 'function') {
      console.error('No move method is provided, cannot be dragged!')

      return
    }

    activeGraph.moveProcessor(e)

    activeGraph.status = 'drag'

    return
  }

  const hoverGraph = graphs.find(graph => graph.status === 'hover')

  const hoverAbleGraphs = graphs.filter(graph =>
    (graph.hover && (typeof graph.hoverCheck === 'function' || graph.hoverRect)))

  const hoveredGraph = hoverAbleGraphs.find(graph => graph.hoverCheckProcessor(position, graph))

  if (hoveredGraph) {
    document.body.style.cursor = hoveredGraph.style.hoverCursor
  } else {
    document.body.style.cursor = 'default'
  }

  let [hoverGraphMouseOuterIsFun, hoveredGraphMouseEnterIsFun] = [false, false]

  if (hoverGraph) hoverGraphMouseOuterIsFun = typeof hoverGraph.mouseOuter === 'function'
  if (hoveredGraph) hoveredGraphMouseEnterIsFun = typeof hoveredGraph.mouseEnter === 'function'

  if (!hoveredGraph && !hoverGraph) return

  if (!hoveredGraph && hoverGraph) {
    if (hoverGraphMouseOuterIsFun) hoverGraph.mouseOuter(e, hoverGraph)

    hoverGraph.status = 'static'

    return
  }

  if (hoveredGraph && hoveredGraph === hoverGraph) return

  if (hoveredGraph && !hoverGraph) {
    if (hoveredGraphMouseEnterIsFun) hoveredGraph.mouseEnter(e, hoveredGraph)

    hoveredGraph.status = 'hover'

    return
  }

  if (hoveredGraph && hoverGraph && hoveredGraph !== hoverGraph) {
    if (hoverGraphMouseOuterIsFun) hoverGraph.mouseOuter(e, hoverGraph)

    hoverGraph.status = 'static'

    if (hoveredGraphMouseEnterIsFun) hoveredGraph.mouseEnter(e, hoveredGraph)

    hoveredGraph.status = 'hover'
  }
}

/**
 * @description Handler of CRender mouseup event
 * @return {Undefined} Void
 */
function mouseUp (e) {
  const { graphs } = this

  const activeGraph = graphs.find(graph => graph.status === 'active')
  const dragGraph = graphs.find(graph => graph.status === 'drag')

  if (activeGraph && typeof activeGraph.click === 'function') activeGraph.click(e, activeGraph)

  graphs.forEach(graph => graph && (graph.status = 'static'))

  if (activeGraph) activeGraph.status = 'hover'
  if (dragGraph) dragGraph.status = 'hover'
}

/**
 * @description         Clone Graph
 * @param {Graph} graph The target to be cloned
 * @return {Graph}      Cloned graph
 */
CRender.prototype.clone = function (graph) {
  const style = graph.style.getStyle()

  let clonedGraph = { ...graph, style }

  delete clonedGraph.render

  clonedGraph = deepClone(clonedGraph, true)

  return this.add(clonedGraph)
}
