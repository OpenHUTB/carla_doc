(function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(require,module,exports){
var transition = require('../lib/index')

window.transition = transition
},{"../lib/index":3}],2:[function(require,module,exports){
"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports["default"] = exports.easeInOutBounce = exports.easeOutBounce = exports.easeInBounce = exports.easeInOutElastic = exports.easeOutElastic = exports.easeInElastic = exports.easeInOutBack = exports.easeOutBack = exports.easeInBack = exports.easeInOutQuint = exports.easeOutQuint = exports.easeInQuint = exports.easeInOutQuart = exports.easeOutQuart = exports.easeInQuart = exports.easeInOutCubic = exports.easeOutCubic = exports.easeInCubic = exports.easeInOutQuad = exports.easeOutQuad = exports.easeInQuad = exports.easeInOutSine = exports.easeOutSine = exports.easeInSine = exports.linear = void 0;
var linear = [[[0, 1], '', [0.33, 0.67]], [[1, 0], [0.67, 0.33]]];
/**
 * @description Sine
 */

exports.linear = linear;
var easeInSine = [[[0, 1]], [[0.538, 0.564], [0.169, 0.912], [0.880, 0.196]], [[1, 0]]];
exports.easeInSine = easeInSine;
var easeOutSine = [[[0, 1]], [[0.444, 0.448], [0.169, 0.736], [0.718, 0.16]], [[1, 0]]];
exports.easeOutSine = easeOutSine;
var easeInOutSine = [[[0, 1]], [[0.5, 0.5], [0.2, 1], [0.8, 0]], [[1, 0]]];
/**
 * @description Quad
 */

exports.easeInOutSine = easeInOutSine;
var easeInQuad = [[[0, 1]], [[0.550, 0.584], [0.231, 0.904], [0.868, 0.264]], [[1, 0]]];
exports.easeInQuad = easeInQuad;
var easeOutQuad = [[[0, 1]], [[0.413, 0.428], [0.065, 0.816], [0.760, 0.04]], [[1, 0]]];
exports.easeOutQuad = easeOutQuad;
var easeInOutQuad = [[[0, 1]], [[0.5, 0.5], [0.3, 0.9], [0.7, 0.1]], [[1, 0]]];
/**
 * @description Cubic
 */

exports.easeInOutQuad = easeInOutQuad;
var easeInCubic = [[[0, 1]], [[0.679, 0.688], [0.366, 0.992], [0.992, 0.384]], [[1, 0]]];
exports.easeInCubic = easeInCubic;
var easeOutCubic = [[[0, 1]], [[0.321, 0.312], [0.008, 0.616], [0.634, 0.008]], [[1, 0]]];
exports.easeOutCubic = easeOutCubic;
var easeInOutCubic = [[[0, 1]], [[0.5, 0.5], [0.3, 1], [0.7, 0]], [[1, 0]]];
/**
 * @description Quart
 */

exports.easeInOutCubic = easeInOutCubic;
var easeInQuart = [[[0, 1]], [[0.812, 0.74], [0.611, 0.988], [1.013, 0.492]], [[1, 0]]];
exports.easeInQuart = easeInQuart;
var easeOutQuart = [[[0, 1]], [[0.152, 0.244], [0.001, 0.448], [0.285, -0.02]], [[1, 0]]];
exports.easeOutQuart = easeOutQuart;
var easeInOutQuart = [[[0, 1]], [[0.5, 0.5], [0.4, 1], [0.6, 0]], [[1, 0]]];
/**
 * @description Quint
 */

exports.easeInOutQuart = easeInOutQuart;
var easeInQuint = [[[0, 1]], [[0.857, 0.856], [0.714, 1], [1, 0.712]], [[1, 0]]];
exports.easeInQuint = easeInQuint;
var easeOutQuint = [[[0, 1]], [[0.108, 0.2], [0.001, 0.4], [0.214, -0.012]], [[1, 0]]];
exports.easeOutQuint = easeOutQuint;
var easeInOutQuint = [[[0, 1]], [[0.5, 0.5], [0.5, 1], [0.5, 0]], [[1, 0]]];
/**
 * @description Back
 */

exports.easeInOutQuint = easeInOutQuint;
var easeInBack = [[[0, 1]], [[0.667, 0.896], [0.380, 1.184], [0.955, 0.616]], [[1, 0]]];
exports.easeInBack = easeInBack;
var easeOutBack = [[[0, 1]], [[0.335, 0.028], [0.061, 0.22], [0.631, -0.18]], [[1, 0]]];
exports.easeOutBack = easeOutBack;
var easeInOutBack = [[[0, 1]], [[0.5, 0.5], [0.4, 1.4], [0.6, -0.4]], [[1, 0]]];
/**
 * @description Elastic
 */

exports.easeInOutBack = easeInOutBack;
var easeInElastic = [[[0, 1]], [[0.474, 0.964], [0.382, 0.988], [0.557, 0.952]], [[0.619, 1.076], [0.565, 1.088], [0.669, 1.08]], [[0.770, 0.916], [0.712, 0.924], [0.847, 0.904]], [[0.911, 1.304], [0.872, 1.316], [0.961, 1.34]], [[1, 0]]];
exports.easeInElastic = easeInElastic;
var easeOutElastic = [[[0, 1]], [[0.073, -0.32], [0.034, -0.328], [0.104, -0.344]], [[0.191, 0.092], [0.110, 0.06], [0.256, 0.08]], [[0.310, -0.076], [0.260, -0.068], [0.357, -0.076]], [[0.432, 0.032], [0.362, 0.028], [0.683, -0.004]], [[1, 0]]];
exports.easeOutElastic = easeOutElastic;
var easeInOutElastic = [[[0, 1]], [[0.210, 0.94], [0.167, 0.884], [0.252, 0.98]], [[0.299, 1.104], [0.256, 1.092], [0.347, 1.108]], [[0.5, 0.496], [0.451, 0.672], [0.548, 0.324]], [[0.696, -0.108], [0.652, -0.112], [0.741, -0.124]], [[0.805, 0.064], [0.756, 0.012], [0.866, 0.096]], [[1, 0]]];
/**
 * @description Bounce
 */

exports.easeInOutElastic = easeInOutElastic;
var easeInBounce = [[[0, 1]], [[0.148, 1], [0.075, 0.868], [0.193, 0.848]], [[0.326, 1], [0.276, 0.836], [0.405, 0.712]], [[0.600, 1], [0.511, 0.708], [0.671, 0.348]], [[1, 0]]];
exports.easeInBounce = easeInBounce;
var easeOutBounce = [[[0, 1]], [[0.357, 0.004], [0.270, 0.592], [0.376, 0.252]], [[0.604, -0.004], [0.548, 0.312], [0.669, 0.184]], [[0.820, 0], [0.749, 0.184], [0.905, 0.132]], [[1, 0]]];
exports.easeOutBounce = easeOutBounce;
var easeInOutBounce = [[[0, 1]], [[0.102, 1], [0.050, 0.864], [0.117, 0.86]], [[0.216, 0.996], [0.208, 0.844], [0.227, 0.808]], [[0.347, 0.996], [0.343, 0.8], [0.480, 0.292]], [[0.635, 0.004], [0.511, 0.676], [0.656, 0.208]], [[0.787, 0], [0.760, 0.2], [0.795, 0.144]], [[0.905, -0.004], [0.899, 0.164], [0.944, 0.144]], [[1, 0]]];
exports.easeInOutBounce = easeInOutBounce;

var _default = new Map([['linear', linear], ['easeInSine', easeInSine], ['easeOutSine', easeOutSine], ['easeInOutSine', easeInOutSine], ['easeInQuad', easeInQuad], ['easeOutQuad', easeOutQuad], ['easeInOutQuad', easeInOutQuad], ['easeInCubic', easeInCubic], ['easeOutCubic', easeOutCubic], ['easeInOutCubic', easeInOutCubic], ['easeInQuart', easeInQuart], ['easeOutQuart', easeOutQuart], ['easeInOutQuart', easeInOutQuart], ['easeInQuint', easeInQuint], ['easeOutQuint', easeOutQuint], ['easeInOutQuint', easeInOutQuint], ['easeInBack', easeInBack], ['easeOutBack', easeOutBack], ['easeInOutBack', easeInOutBack], ['easeInElastic', easeInElastic], ['easeOutElastic', easeOutElastic], ['easeInOutElastic', easeInOutElastic], ['easeInBounce', easeInBounce], ['easeOutBounce', easeOutBounce], ['easeInOutBounce', easeInOutBounce]]);

exports["default"] = _default;
},{}],3:[function(require,module,exports){
"use strict";

var _interopRequireDefault = require("@babel/runtime/helpers/interopRequireDefault");

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.transition = transition;
exports.injectNewCurve = injectNewCurve;
exports["default"] = void 0;

var _slicedToArray2 = _interopRequireDefault(require("@babel/runtime/helpers/slicedToArray"));

var _typeof2 = _interopRequireDefault(require("@babel/runtime/helpers/typeof"));

var _curves = _interopRequireDefault(require("./config/curves"));

var defaultTransitionBC = 'linear';
/**
 * @description Get the N-frame animation state by the start and end state
 *              of the animation and the easing curve
 * @param {String|Array} tBC               Easing curve name or data
 * @param {Number|Array|Object} startState Animation start state
 * @param {Number|Array|Object} endState   Animation end state
 * @param {Number} frameNum                Number of Animation frames
 * @param {Boolean} deep                   Whether to use recursive mode
 * @return {Array|Boolean} State of each frame of the animation (Invalid input will return false)
 */

function transition(tBC) {
  var startState = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;
  var endState = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : null;
  var frameNum = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 30;
  var deep = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : false;
  if (!checkParams.apply(void 0, arguments)) return false;

  try {
    // Get the transition bezier curve
    var bezierCurve = getBezierCurve(tBC); // Get the progress of each frame state

    var frameStateProgress = getFrameStateProgress(bezierCurve, frameNum); // If the recursion mode is not enabled or the state type is Number, the shallow state calculation is performed directly.

    if (!deep || typeof endState === 'number') return getTransitionState(startState, endState, frameStateProgress);
    return recursionTransitionState(startState, endState, frameStateProgress);
  } catch (_unused) {
    console.warn('Transition parameter may be abnormal!');
    return [endState];
  }
}
/**
 * @description Check if the parameters are legal
 * @param {String} tBC      Name of transition bezier curve
 * @param {Any} startState  Transition start state
 * @param {Any} endState    Transition end state
 * @param {Number} frameNum Number of transition frames
 * @return {Boolean} Is the parameter legal
 */


function checkParams(tBC) {
  var startState = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;
  var endState = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : false;
  var frameNum = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : 30;

  if (!tBC || startState === false || endState === false || !frameNum) {
    console.error('transition: Missing Parameters!');
    return false;
  }

  if ((0, _typeof2["default"])(startState) !== (0, _typeof2["default"])(endState)) {
    console.error('transition: Inconsistent Status Types!');
    return false;
  }

  var stateType = (0, _typeof2["default"])(endState);

  if (stateType === 'string' || stateType === 'boolean' || !tBC.length) {
    console.error('transition: Unsupported Data Type of State!');
    return false;
  }

  if (!_curves["default"].has(tBC) && !(tBC instanceof Array)) {
    console.warn('transition: Transition curve not found, default curve will be used!');
  }

  return true;
}
/**
 * @description Get the transition bezier curve
 * @param {String} tBC Name of transition bezier curve
 * @return {Array} Bezier curve data
 */


function getBezierCurve(tBC) {
  var bezierCurve = '';

  if (_curves["default"].has(tBC)) {
    bezierCurve = _curves["default"].get(tBC);
  } else if (tBC instanceof Array) {
    bezierCurve = tBC;
  } else {
    bezierCurve = _curves["default"].get(defaultTransitionBC);
  }

  return bezierCurve;
}
/**
 * @description Get the progress of each frame state
 * @param {Array} bezierCurve Transition bezier curve
 * @param {Number} frameNum   Number of transition frames
 * @return {Array} Progress of each frame state
 */


function getFrameStateProgress(bezierCurve, frameNum) {
  var tMinus = 1 / (frameNum - 1);
  var tState = new Array(frameNum).fill(0).map(function (t, i) {
    return i * tMinus;
  });
  var frameState = tState.map(function (t) {
    return getFrameStateFromT(bezierCurve, t);
  });
  return frameState;
}
/**
 * @description Get the progress of the corresponding frame according to t
 * @param {Array} bezierCurve Transition bezier curve
 * @param {Number} t          Current frame t
 * @return {Number} Progress of current frame
 */


function getFrameStateFromT(bezierCurve, t) {
  var tBezierCurvePoint = getBezierCurvePointFromT(bezierCurve, t);
  var bezierCurvePointT = getBezierCurvePointTFromReT(tBezierCurvePoint, t);
  return getBezierCurveTState(tBezierCurvePoint, bezierCurvePointT);
}
/**
 * @description Get the corresponding sub-curve according to t
 * @param {Array} bezierCurve Transition bezier curve
 * @param {Number} t          Current frame t
 * @return {Array} Sub-curve of t
 */


function getBezierCurvePointFromT(bezierCurve, t) {
  var lastIndex = bezierCurve.length - 1;
  var begin = '',
      end = '';
  bezierCurve.findIndex(function (item, i) {
    if (i === lastIndex) return;
    begin = item;
    end = bezierCurve[i + 1];
    var currentMainPointX = begin[0][0];
    var nextMainPointX = end[0][0];
    return t >= currentMainPointX && t < nextMainPointX;
  });
  var p0 = begin[0];
  var p1 = begin[2] || begin[0];
  var p2 = end[1] || end[0];
  var p3 = end[0];
  return [p0, p1, p2, p3];
}
/**
 * @description Get local t based on t and sub-curve
 * @param {Array} bezierCurve Sub-curve
 * @param {Number} t          Current frame t
 * @return {Number} local t of sub-curve
 */


function getBezierCurvePointTFromReT(bezierCurve, t) {
  var reBeginX = bezierCurve[0][0];
  var reEndX = bezierCurve[3][0];
  var xMinus = reEndX - reBeginX;
  var tMinus = t - reBeginX;
  return tMinus / xMinus;
}
/**
 * @description Get the curve progress of t
 * @param {Array} bezierCurve Sub-curve
 * @param {Number} t          Current frame t
 * @return {Number} Progress of current frame
 */


function getBezierCurveTState(_ref, t) {
  var _ref2 = (0, _slicedToArray2["default"])(_ref, 4),
      _ref2$ = (0, _slicedToArray2["default"])(_ref2[0], 2),
      p0 = _ref2$[1],
      _ref2$2 = (0, _slicedToArray2["default"])(_ref2[1], 2),
      p1 = _ref2$2[1],
      _ref2$3 = (0, _slicedToArray2["default"])(_ref2[2], 2),
      p2 = _ref2$3[1],
      _ref2$4 = (0, _slicedToArray2["default"])(_ref2[3], 2),
      p3 = _ref2$4[1];

  var pow = Math.pow;
  var tMinus = 1 - t;
  var result1 = p0 * pow(tMinus, 3);
  var result2 = 3 * p1 * t * pow(tMinus, 2);
  var result3 = 3 * p2 * pow(t, 2) * tMinus;
  var result4 = p3 * pow(t, 3);
  return 1 - (result1 + result2 + result3 + result4);
}
/**
 * @description Get transition state according to frame progress
 * @param {Any} startState   Transition start state
 * @param {Any} endState     Transition end state
 * @param {Array} frameState Frame state progress
 * @return {Array} Transition frame state
 */


function getTransitionState(begin, end, frameState) {
  var stateType = 'object';
  if (typeof begin === 'number') stateType = 'number';
  if (begin instanceof Array) stateType = 'array';
  if (stateType === 'number') return getNumberTransitionState(begin, end, frameState);
  if (stateType === 'array') return getArrayTransitionState(begin, end, frameState);
  if (stateType === 'object') return getObjectTransitionState(begin, end, frameState);
  return frameState.map(function (t) {
    return end;
  });
}
/**
 * @description Get the transition data of the number type
 * @param {Number} startState Transition start state
 * @param {Number} endState   Transition end state
 * @param {Array} frameState  Frame state progress
 * @return {Array} Transition frame state
 */


function getNumberTransitionState(begin, end, frameState) {
  var minus = end - begin;
  return frameState.map(function (s) {
    return begin + minus * s;
  });
}
/**
 * @description Get the transition data of the array type
 * @param {Array} startState Transition start state
 * @param {Array} endState   Transition end state
 * @param {Array} frameState Frame state progress
 * @return {Array} Transition frame state
 */


function getArrayTransitionState(begin, end, frameState) {
  var minus = end.map(function (v, i) {
    if (typeof v !== 'number') return false;
    return v - begin[i];
  });
  return frameState.map(function (s) {
    return minus.map(function (v, i) {
      if (v === false) return end[i];
      return begin[i] + v * s;
    });
  });
}
/**
 * @description Get the transition data of the object type
 * @param {Object} startState Transition start state
 * @param {Object} endState   Transition end state
 * @param {Array} frameState  Frame state progress
 * @return {Array} Transition frame state
 */


function getObjectTransitionState(begin, end, frameState) {
  var keys = Object.keys(end);
  var beginValue = keys.map(function (k) {
    return begin[k];
  });
  var endValue = keys.map(function (k) {
    return end[k];
  });
  var arrayState = getArrayTransitionState(beginValue, endValue, frameState);
  return arrayState.map(function (item) {
    var frameData = {};
    item.forEach(function (v, i) {
      return frameData[keys[i]] = v;
    });
    return frameData;
  });
}
/**
 * @description Get the transition state data by recursion
 * @param {Array|Object} startState Transition start state
 * @param {Array|Object} endState   Transition end state
 * @param {Array} frameState        Frame state progress
 * @return {Array} Transition frame state
 */


function recursionTransitionState(begin, end, frameState) {
  var state = getTransitionState(begin, end, frameState);

  var _loop = function _loop(key) {
    var bTemp = begin[key];
    var eTemp = end[key];
    if ((0, _typeof2["default"])(eTemp) !== 'object') return "continue";
    var data = recursionTransitionState(bTemp, eTemp, frameState);
    state.forEach(function (fs, i) {
      return fs[key] = data[i];
    });
  };

  for (var key in end) {
    var _ret = _loop(key);

    if (_ret === "continue") continue;
  }

  return state;
}
/**
 * @description Inject new curve into curves as config
 * @param {Any} key     The key of curve
 * @param {Array} curve Bezier curve data
 * @return {Undefined} No return
 */


function injectNewCurve(key, curve) {
  if (!key || !curve) {
    console.error('InjectNewCurve Missing Parameters!');
    return;
  }

  _curves["default"].set(key, curve);
}

var _default = transition;
exports["default"] = _default;
},{"./config/curves":2,"@babel/runtime/helpers/interopRequireDefault":5,"@babel/runtime/helpers/slicedToArray":8,"@babel/runtime/helpers/typeof":9}],4:[function(require,module,exports){
function _arrayWithHoles(arr) {
  if (Array.isArray(arr)) return arr;
}

module.exports = _arrayWithHoles;
},{}],5:[function(require,module,exports){
function _interopRequireDefault(obj) {
  return obj && obj.__esModule ? obj : {
    "default": obj
  };
}

module.exports = _interopRequireDefault;
},{}],6:[function(require,module,exports){
function _iterableToArrayLimit(arr, i) {
  var _arr = [];
  var _n = true;
  var _d = false;
  var _e = undefined;

  try {
    for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) {
      _arr.push(_s.value);

      if (i && _arr.length === i) break;
    }
  } catch (err) {
    _d = true;
    _e = err;
  } finally {
    try {
      if (!_n && _i["return"] != null) _i["return"]();
    } finally {
      if (_d) throw _e;
    }
  }

  return _arr;
}

module.exports = _iterableToArrayLimit;
},{}],7:[function(require,module,exports){
function _nonIterableRest() {
  throw new TypeError("Invalid attempt to destructure non-iterable instance");
}

module.exports = _nonIterableRest;
},{}],8:[function(require,module,exports){
var arrayWithHoles = require("./arrayWithHoles");

var iterableToArrayLimit = require("./iterableToArrayLimit");

var nonIterableRest = require("./nonIterableRest");

function _slicedToArray(arr, i) {
  return arrayWithHoles(arr) || iterableToArrayLimit(arr, i) || nonIterableRest();
}

module.exports = _slicedToArray;
},{"./arrayWithHoles":4,"./iterableToArrayLimit":6,"./nonIterableRest":7}],9:[function(require,module,exports){
function _typeof2(obj) { if (typeof Symbol === "function" && typeof Symbol.iterator === "symbol") { _typeof2 = function _typeof2(obj) { return typeof obj; }; } else { _typeof2 = function _typeof2(obj) { return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; }; } return _typeof2(obj); }

function _typeof(obj) {
  if (typeof Symbol === "function" && _typeof2(Symbol.iterator) === "symbol") {
    module.exports = _typeof = function _typeof(obj) {
      return _typeof2(obj);
    };
  } else {
    module.exports = _typeof = function _typeof(obj) {
      return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : _typeof2(obj);
    };
  }

  return _typeof(obj);
}

module.exports = _typeof;
},{}]},{},[1])
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIkM6L1VzZXJzL0FkbWluaXN0cmF0b3IvQXBwRGF0YS9Sb2FtaW5nL25wbS9ub2RlX21vZHVsZXMvYnJvd3NlcmlmeS9ub2RlX21vZHVsZXMvYnJvd3Nlci1wYWNrL19wcmVsdWRlLmpzIiwiYnVpbGQvZW50cnkuanMiLCJsaWIvY29uZmlnL2N1cnZlcy5qcyIsImxpYi9pbmRleC5qcyIsIm5vZGVfbW9kdWxlcy9AYmFiZWwvcnVudGltZS9oZWxwZXJzL2FycmF5V2l0aEhvbGVzLmpzIiwibm9kZV9tb2R1bGVzL0BiYWJlbC9ydW50aW1lL2hlbHBlcnMvaW50ZXJvcFJlcXVpcmVEZWZhdWx0LmpzIiwibm9kZV9tb2R1bGVzL0BiYWJlbC9ydW50aW1lL2hlbHBlcnMvaXRlcmFibGVUb0FycmF5TGltaXQuanMiLCJub2RlX21vZHVsZXMvQGJhYmVsL3J1bnRpbWUvaGVscGVycy9ub25JdGVyYWJsZVJlc3QuanMiLCJub2RlX21vZHVsZXMvQGJhYmVsL3J1bnRpbWUvaGVscGVycy9zbGljZWRUb0FycmF5LmpzIiwibm9kZV9tb2R1bGVzL0BiYWJlbC9ydW50aW1lL2hlbHBlcnMvdHlwZW9mLmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0FDQUE7QUFDQTtBQUNBOztBQ0ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDM0ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2hWQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0pBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ05BO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMxQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ1ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbigpe2Z1bmN0aW9uIHIoZSxuLHQpe2Z1bmN0aW9uIG8oaSxmKXtpZighbltpXSl7aWYoIWVbaV0pe3ZhciBjPVwiZnVuY3Rpb25cIj09dHlwZW9mIHJlcXVpcmUmJnJlcXVpcmU7aWYoIWYmJmMpcmV0dXJuIGMoaSwhMCk7aWYodSlyZXR1cm4gdShpLCEwKTt2YXIgYT1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK2krXCInXCIpO3Rocm93IGEuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixhfXZhciBwPW5baV09e2V4cG9ydHM6e319O2VbaV1bMF0uY2FsbChwLmV4cG9ydHMsZnVuY3Rpb24ocil7dmFyIG49ZVtpXVsxXVtyXTtyZXR1cm4gbyhufHxyKX0scCxwLmV4cG9ydHMscixlLG4sdCl9cmV0dXJuIG5baV0uZXhwb3J0c31mb3IodmFyIHU9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZSxpPTA7aTx0Lmxlbmd0aDtpKyspbyh0W2ldKTtyZXR1cm4gb31yZXR1cm4gcn0pKCkiLCJ2YXIgdHJhbnNpdGlvbiA9IHJlcXVpcmUoJy4uL2xpYi9pbmRleCcpXHJcblxyXG53aW5kb3cudHJhbnNpdGlvbiA9IHRyYW5zaXRpb24iLCJcInVzZSBzdHJpY3RcIjtcblxuT2JqZWN0LmRlZmluZVByb3BlcnR5KGV4cG9ydHMsIFwiX19lc01vZHVsZVwiLCB7XG4gIHZhbHVlOiB0cnVlXG59KTtcbmV4cG9ydHNbXCJkZWZhdWx0XCJdID0gZXhwb3J0cy5lYXNlSW5PdXRCb3VuY2UgPSBleHBvcnRzLmVhc2VPdXRCb3VuY2UgPSBleHBvcnRzLmVhc2VJbkJvdW5jZSA9IGV4cG9ydHMuZWFzZUluT3V0RWxhc3RpYyA9IGV4cG9ydHMuZWFzZU91dEVsYXN0aWMgPSBleHBvcnRzLmVhc2VJbkVsYXN0aWMgPSBleHBvcnRzLmVhc2VJbk91dEJhY2sgPSBleHBvcnRzLmVhc2VPdXRCYWNrID0gZXhwb3J0cy5lYXNlSW5CYWNrID0gZXhwb3J0cy5lYXNlSW5PdXRRdWludCA9IGV4cG9ydHMuZWFzZU91dFF1aW50ID0gZXhwb3J0cy5lYXNlSW5RdWludCA9IGV4cG9ydHMuZWFzZUluT3V0UXVhcnQgPSBleHBvcnRzLmVhc2VPdXRRdWFydCA9IGV4cG9ydHMuZWFzZUluUXVhcnQgPSBleHBvcnRzLmVhc2VJbk91dEN1YmljID0gZXhwb3J0cy5lYXNlT3V0Q3ViaWMgPSBleHBvcnRzLmVhc2VJbkN1YmljID0gZXhwb3J0cy5lYXNlSW5PdXRRdWFkID0gZXhwb3J0cy5lYXNlT3V0UXVhZCA9IGV4cG9ydHMuZWFzZUluUXVhZCA9IGV4cG9ydHMuZWFzZUluT3V0U2luZSA9IGV4cG9ydHMuZWFzZU91dFNpbmUgPSBleHBvcnRzLmVhc2VJblNpbmUgPSBleHBvcnRzLmxpbmVhciA9IHZvaWQgMDtcbnZhciBsaW5lYXIgPSBbW1swLCAxXSwgJycsIFswLjMzLCAwLjY3XV0sIFtbMSwgMF0sIFswLjY3LCAwLjMzXV1dO1xuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBTaW5lXHJcbiAqL1xuXG5leHBvcnRzLmxpbmVhciA9IGxpbmVhcjtcbnZhciBlYXNlSW5TaW5lID0gW1tbMCwgMV1dLCBbWzAuNTM4LCAwLjU2NF0sIFswLjE2OSwgMC45MTJdLCBbMC44ODAsIDAuMTk2XV0sIFtbMSwgMF1dXTtcbmV4cG9ydHMuZWFzZUluU2luZSA9IGVhc2VJblNpbmU7XG52YXIgZWFzZU91dFNpbmUgPSBbW1swLCAxXV0sIFtbMC40NDQsIDAuNDQ4XSwgWzAuMTY5LCAwLjczNl0sIFswLjcxOCwgMC4xNl1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VPdXRTaW5lID0gZWFzZU91dFNpbmU7XG52YXIgZWFzZUluT3V0U2luZSA9IFtbWzAsIDFdXSwgW1swLjUsIDAuNV0sIFswLjIsIDFdLCBbMC44LCAwXV0sIFtbMSwgMF1dXTtcbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gUXVhZFxyXG4gKi9cblxuZXhwb3J0cy5lYXNlSW5PdXRTaW5lID0gZWFzZUluT3V0U2luZTtcbnZhciBlYXNlSW5RdWFkID0gW1tbMCwgMV1dLCBbWzAuNTUwLCAwLjU4NF0sIFswLjIzMSwgMC45MDRdLCBbMC44NjgsIDAuMjY0XV0sIFtbMSwgMF1dXTtcbmV4cG9ydHMuZWFzZUluUXVhZCA9IGVhc2VJblF1YWQ7XG52YXIgZWFzZU91dFF1YWQgPSBbW1swLCAxXV0sIFtbMC40MTMsIDAuNDI4XSwgWzAuMDY1LCAwLjgxNl0sIFswLjc2MCwgMC4wNF1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VPdXRRdWFkID0gZWFzZU91dFF1YWQ7XG52YXIgZWFzZUluT3V0UXVhZCA9IFtbWzAsIDFdXSwgW1swLjUsIDAuNV0sIFswLjMsIDAuOV0sIFswLjcsIDAuMV1dLCBbWzEsIDBdXV07XG4vKipcclxuICogQGRlc2NyaXB0aW9uIEN1YmljXHJcbiAqL1xuXG5leHBvcnRzLmVhc2VJbk91dFF1YWQgPSBlYXNlSW5PdXRRdWFkO1xudmFyIGVhc2VJbkN1YmljID0gW1tbMCwgMV1dLCBbWzAuNjc5LCAwLjY4OF0sIFswLjM2NiwgMC45OTJdLCBbMC45OTIsIDAuMzg0XV0sIFtbMSwgMF1dXTtcbmV4cG9ydHMuZWFzZUluQ3ViaWMgPSBlYXNlSW5DdWJpYztcbnZhciBlYXNlT3V0Q3ViaWMgPSBbW1swLCAxXV0sIFtbMC4zMjEsIDAuMzEyXSwgWzAuMDA4LCAwLjYxNl0sIFswLjYzNCwgMC4wMDhdXSwgW1sxLCAwXV1dO1xuZXhwb3J0cy5lYXNlT3V0Q3ViaWMgPSBlYXNlT3V0Q3ViaWM7XG52YXIgZWFzZUluT3V0Q3ViaWMgPSBbW1swLCAxXV0sIFtbMC41LCAwLjVdLCBbMC4zLCAxXSwgWzAuNywgMF1dLCBbWzEsIDBdXV07XG4vKipcclxuICogQGRlc2NyaXB0aW9uIFF1YXJ0XHJcbiAqL1xuXG5leHBvcnRzLmVhc2VJbk91dEN1YmljID0gZWFzZUluT3V0Q3ViaWM7XG52YXIgZWFzZUluUXVhcnQgPSBbW1swLCAxXV0sIFtbMC44MTIsIDAuNzRdLCBbMC42MTEsIDAuOTg4XSwgWzEuMDEzLCAwLjQ5Ml1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VJblF1YXJ0ID0gZWFzZUluUXVhcnQ7XG52YXIgZWFzZU91dFF1YXJ0ID0gW1tbMCwgMV1dLCBbWzAuMTUyLCAwLjI0NF0sIFswLjAwMSwgMC40NDhdLCBbMC4yODUsIC0wLjAyXV0sIFtbMSwgMF1dXTtcbmV4cG9ydHMuZWFzZU91dFF1YXJ0ID0gZWFzZU91dFF1YXJ0O1xudmFyIGVhc2VJbk91dFF1YXJ0ID0gW1tbMCwgMV1dLCBbWzAuNSwgMC41XSwgWzAuNCwgMV0sIFswLjYsIDBdXSwgW1sxLCAwXV1dO1xuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBRdWludFxyXG4gKi9cblxuZXhwb3J0cy5lYXNlSW5PdXRRdWFydCA9IGVhc2VJbk91dFF1YXJ0O1xudmFyIGVhc2VJblF1aW50ID0gW1tbMCwgMV1dLCBbWzAuODU3LCAwLjg1Nl0sIFswLjcxNCwgMV0sIFsxLCAwLjcxMl1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VJblF1aW50ID0gZWFzZUluUXVpbnQ7XG52YXIgZWFzZU91dFF1aW50ID0gW1tbMCwgMV1dLCBbWzAuMTA4LCAwLjJdLCBbMC4wMDEsIDAuNF0sIFswLjIxNCwgLTAuMDEyXV0sIFtbMSwgMF1dXTtcbmV4cG9ydHMuZWFzZU91dFF1aW50ID0gZWFzZU91dFF1aW50O1xudmFyIGVhc2VJbk91dFF1aW50ID0gW1tbMCwgMV1dLCBbWzAuNSwgMC41XSwgWzAuNSwgMV0sIFswLjUsIDBdXSwgW1sxLCAwXV1dO1xuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBCYWNrXHJcbiAqL1xuXG5leHBvcnRzLmVhc2VJbk91dFF1aW50ID0gZWFzZUluT3V0UXVpbnQ7XG52YXIgZWFzZUluQmFjayA9IFtbWzAsIDFdXSwgW1swLjY2NywgMC44OTZdLCBbMC4zODAsIDEuMTg0XSwgWzAuOTU1LCAwLjYxNl1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VJbkJhY2sgPSBlYXNlSW5CYWNrO1xudmFyIGVhc2VPdXRCYWNrID0gW1tbMCwgMV1dLCBbWzAuMzM1LCAwLjAyOF0sIFswLjA2MSwgMC4yMl0sIFswLjYzMSwgLTAuMThdXSwgW1sxLCAwXV1dO1xuZXhwb3J0cy5lYXNlT3V0QmFjayA9IGVhc2VPdXRCYWNrO1xudmFyIGVhc2VJbk91dEJhY2sgPSBbW1swLCAxXV0sIFtbMC41LCAwLjVdLCBbMC40LCAxLjRdLCBbMC42LCAtMC40XV0sIFtbMSwgMF1dXTtcbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gRWxhc3RpY1xyXG4gKi9cblxuZXhwb3J0cy5lYXNlSW5PdXRCYWNrID0gZWFzZUluT3V0QmFjaztcbnZhciBlYXNlSW5FbGFzdGljID0gW1tbMCwgMV1dLCBbWzAuNDc0LCAwLjk2NF0sIFswLjM4MiwgMC45ODhdLCBbMC41NTcsIDAuOTUyXV0sIFtbMC42MTksIDEuMDc2XSwgWzAuNTY1LCAxLjA4OF0sIFswLjY2OSwgMS4wOF1dLCBbWzAuNzcwLCAwLjkxNl0sIFswLjcxMiwgMC45MjRdLCBbMC44NDcsIDAuOTA0XV0sIFtbMC45MTEsIDEuMzA0XSwgWzAuODcyLCAxLjMxNl0sIFswLjk2MSwgMS4zNF1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VJbkVsYXN0aWMgPSBlYXNlSW5FbGFzdGljO1xudmFyIGVhc2VPdXRFbGFzdGljID0gW1tbMCwgMV1dLCBbWzAuMDczLCAtMC4zMl0sIFswLjAzNCwgLTAuMzI4XSwgWzAuMTA0LCAtMC4zNDRdXSwgW1swLjE5MSwgMC4wOTJdLCBbMC4xMTAsIDAuMDZdLCBbMC4yNTYsIDAuMDhdXSwgW1swLjMxMCwgLTAuMDc2XSwgWzAuMjYwLCAtMC4wNjhdLCBbMC4zNTcsIC0wLjA3Nl1dLCBbWzAuNDMyLCAwLjAzMl0sIFswLjM2MiwgMC4wMjhdLCBbMC42ODMsIC0wLjAwNF1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VPdXRFbGFzdGljID0gZWFzZU91dEVsYXN0aWM7XG52YXIgZWFzZUluT3V0RWxhc3RpYyA9IFtbWzAsIDFdXSwgW1swLjIxMCwgMC45NF0sIFswLjE2NywgMC44ODRdLCBbMC4yNTIsIDAuOThdXSwgW1swLjI5OSwgMS4xMDRdLCBbMC4yNTYsIDEuMDkyXSwgWzAuMzQ3LCAxLjEwOF1dLCBbWzAuNSwgMC40OTZdLCBbMC40NTEsIDAuNjcyXSwgWzAuNTQ4LCAwLjMyNF1dLCBbWzAuNjk2LCAtMC4xMDhdLCBbMC42NTIsIC0wLjExMl0sIFswLjc0MSwgLTAuMTI0XV0sIFtbMC44MDUsIDAuMDY0XSwgWzAuNzU2LCAwLjAxMl0sIFswLjg2NiwgMC4wOTZdXSwgW1sxLCAwXV1dO1xuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBCb3VuY2VcclxuICovXG5cbmV4cG9ydHMuZWFzZUluT3V0RWxhc3RpYyA9IGVhc2VJbk91dEVsYXN0aWM7XG52YXIgZWFzZUluQm91bmNlID0gW1tbMCwgMV1dLCBbWzAuMTQ4LCAxXSwgWzAuMDc1LCAwLjg2OF0sIFswLjE5MywgMC44NDhdXSwgW1swLjMyNiwgMV0sIFswLjI3NiwgMC44MzZdLCBbMC40MDUsIDAuNzEyXV0sIFtbMC42MDAsIDFdLCBbMC41MTEsIDAuNzA4XSwgWzAuNjcxLCAwLjM0OF1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VJbkJvdW5jZSA9IGVhc2VJbkJvdW5jZTtcbnZhciBlYXNlT3V0Qm91bmNlID0gW1tbMCwgMV1dLCBbWzAuMzU3LCAwLjAwNF0sIFswLjI3MCwgMC41OTJdLCBbMC4zNzYsIDAuMjUyXV0sIFtbMC42MDQsIC0wLjAwNF0sIFswLjU0OCwgMC4zMTJdLCBbMC42NjksIDAuMTg0XV0sIFtbMC44MjAsIDBdLCBbMC43NDksIDAuMTg0XSwgWzAuOTA1LCAwLjEzMl1dLCBbWzEsIDBdXV07XG5leHBvcnRzLmVhc2VPdXRCb3VuY2UgPSBlYXNlT3V0Qm91bmNlO1xudmFyIGVhc2VJbk91dEJvdW5jZSA9IFtbWzAsIDFdXSwgW1swLjEwMiwgMV0sIFswLjA1MCwgMC44NjRdLCBbMC4xMTcsIDAuODZdXSwgW1swLjIxNiwgMC45OTZdLCBbMC4yMDgsIDAuODQ0XSwgWzAuMjI3LCAwLjgwOF1dLCBbWzAuMzQ3LCAwLjk5Nl0sIFswLjM0MywgMC44XSwgWzAuNDgwLCAwLjI5Ml1dLCBbWzAuNjM1LCAwLjAwNF0sIFswLjUxMSwgMC42NzZdLCBbMC42NTYsIDAuMjA4XV0sIFtbMC43ODcsIDBdLCBbMC43NjAsIDAuMl0sIFswLjc5NSwgMC4xNDRdXSwgW1swLjkwNSwgLTAuMDA0XSwgWzAuODk5LCAwLjE2NF0sIFswLjk0NCwgMC4xNDRdXSwgW1sxLCAwXV1dO1xuZXhwb3J0cy5lYXNlSW5PdXRCb3VuY2UgPSBlYXNlSW5PdXRCb3VuY2U7XG5cbnZhciBfZGVmYXVsdCA9IG5ldyBNYXAoW1snbGluZWFyJywgbGluZWFyXSwgWydlYXNlSW5TaW5lJywgZWFzZUluU2luZV0sIFsnZWFzZU91dFNpbmUnLCBlYXNlT3V0U2luZV0sIFsnZWFzZUluT3V0U2luZScsIGVhc2VJbk91dFNpbmVdLCBbJ2Vhc2VJblF1YWQnLCBlYXNlSW5RdWFkXSwgWydlYXNlT3V0UXVhZCcsIGVhc2VPdXRRdWFkXSwgWydlYXNlSW5PdXRRdWFkJywgZWFzZUluT3V0UXVhZF0sIFsnZWFzZUluQ3ViaWMnLCBlYXNlSW5DdWJpY10sIFsnZWFzZU91dEN1YmljJywgZWFzZU91dEN1YmljXSwgWydlYXNlSW5PdXRDdWJpYycsIGVhc2VJbk91dEN1YmljXSwgWydlYXNlSW5RdWFydCcsIGVhc2VJblF1YXJ0XSwgWydlYXNlT3V0UXVhcnQnLCBlYXNlT3V0UXVhcnRdLCBbJ2Vhc2VJbk91dFF1YXJ0JywgZWFzZUluT3V0UXVhcnRdLCBbJ2Vhc2VJblF1aW50JywgZWFzZUluUXVpbnRdLCBbJ2Vhc2VPdXRRdWludCcsIGVhc2VPdXRRdWludF0sIFsnZWFzZUluT3V0UXVpbnQnLCBlYXNlSW5PdXRRdWludF0sIFsnZWFzZUluQmFjaycsIGVhc2VJbkJhY2tdLCBbJ2Vhc2VPdXRCYWNrJywgZWFzZU91dEJhY2tdLCBbJ2Vhc2VJbk91dEJhY2snLCBlYXNlSW5PdXRCYWNrXSwgWydlYXNlSW5FbGFzdGljJywgZWFzZUluRWxhc3RpY10sIFsnZWFzZU91dEVsYXN0aWMnLCBlYXNlT3V0RWxhc3RpY10sIFsnZWFzZUluT3V0RWxhc3RpYycsIGVhc2VJbk91dEVsYXN0aWNdLCBbJ2Vhc2VJbkJvdW5jZScsIGVhc2VJbkJvdW5jZV0sIFsnZWFzZU91dEJvdW5jZScsIGVhc2VPdXRCb3VuY2VdLCBbJ2Vhc2VJbk91dEJvdW5jZScsIGVhc2VJbk91dEJvdW5jZV1dKTtcblxuZXhwb3J0c1tcImRlZmF1bHRcIl0gPSBfZGVmYXVsdDsiLCJcInVzZSBzdHJpY3RcIjtcblxudmFyIF9pbnRlcm9wUmVxdWlyZURlZmF1bHQgPSByZXF1aXJlKFwiQGJhYmVsL3J1bnRpbWUvaGVscGVycy9pbnRlcm9wUmVxdWlyZURlZmF1bHRcIik7XG5cbk9iamVjdC5kZWZpbmVQcm9wZXJ0eShleHBvcnRzLCBcIl9fZXNNb2R1bGVcIiwge1xuICB2YWx1ZTogdHJ1ZVxufSk7XG5leHBvcnRzLnRyYW5zaXRpb24gPSB0cmFuc2l0aW9uO1xuZXhwb3J0cy5pbmplY3ROZXdDdXJ2ZSA9IGluamVjdE5ld0N1cnZlO1xuZXhwb3J0c1tcImRlZmF1bHRcIl0gPSB2b2lkIDA7XG5cbnZhciBfc2xpY2VkVG9BcnJheTIgPSBfaW50ZXJvcFJlcXVpcmVEZWZhdWx0KHJlcXVpcmUoXCJAYmFiZWwvcnVudGltZS9oZWxwZXJzL3NsaWNlZFRvQXJyYXlcIikpO1xuXG52YXIgX3R5cGVvZjIgPSBfaW50ZXJvcFJlcXVpcmVEZWZhdWx0KHJlcXVpcmUoXCJAYmFiZWwvcnVudGltZS9oZWxwZXJzL3R5cGVvZlwiKSk7XG5cbnZhciBfY3VydmVzID0gX2ludGVyb3BSZXF1aXJlRGVmYXVsdChyZXF1aXJlKFwiLi9jb25maWcvY3VydmVzXCIpKTtcblxudmFyIGRlZmF1bHRUcmFuc2l0aW9uQkMgPSAnbGluZWFyJztcbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gR2V0IHRoZSBOLWZyYW1lIGFuaW1hdGlvbiBzdGF0ZSBieSB0aGUgc3RhcnQgYW5kIGVuZCBzdGF0ZVxyXG4gKiAgICAgICAgICAgICAgb2YgdGhlIGFuaW1hdGlvbiBhbmQgdGhlIGVhc2luZyBjdXJ2ZVxyXG4gKiBAcGFyYW0ge1N0cmluZ3xBcnJheX0gdEJDICAgICAgICAgICAgICAgRWFzaW5nIGN1cnZlIG5hbWUgb3IgZGF0YVxyXG4gKiBAcGFyYW0ge051bWJlcnxBcnJheXxPYmplY3R9IHN0YXJ0U3RhdGUgQW5pbWF0aW9uIHN0YXJ0IHN0YXRlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfEFycmF5fE9iamVjdH0gZW5kU3RhdGUgICBBbmltYXRpb24gZW5kIHN0YXRlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSBmcmFtZU51bSAgICAgICAgICAgICAgICBOdW1iZXIgb2YgQW5pbWF0aW9uIGZyYW1lc1xyXG4gKiBAcGFyYW0ge0Jvb2xlYW59IGRlZXAgICAgICAgICAgICAgICAgICAgV2hldGhlciB0byB1c2UgcmVjdXJzaXZlIG1vZGVcclxuICogQHJldHVybiB7QXJyYXl8Qm9vbGVhbn0gU3RhdGUgb2YgZWFjaCBmcmFtZSBvZiB0aGUgYW5pbWF0aW9uIChJbnZhbGlkIGlucHV0IHdpbGwgcmV0dXJuIGZhbHNlKVxyXG4gKi9cblxuZnVuY3Rpb24gdHJhbnNpdGlvbih0QkMpIHtcbiAgdmFyIHN0YXJ0U3RhdGUgPSBhcmd1bWVudHMubGVuZ3RoID4gMSAmJiBhcmd1bWVudHNbMV0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1sxXSA6IG51bGw7XG4gIHZhciBlbmRTdGF0ZSA9IGFyZ3VtZW50cy5sZW5ndGggPiAyICYmIGFyZ3VtZW50c1syXSAhPT0gdW5kZWZpbmVkID8gYXJndW1lbnRzWzJdIDogbnVsbDtcbiAgdmFyIGZyYW1lTnVtID0gYXJndW1lbnRzLmxlbmd0aCA+IDMgJiYgYXJndW1lbnRzWzNdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbM10gOiAzMDtcbiAgdmFyIGRlZXAgPSBhcmd1bWVudHMubGVuZ3RoID4gNCAmJiBhcmd1bWVudHNbNF0gIT09IHVuZGVmaW5lZCA/IGFyZ3VtZW50c1s0XSA6IGZhbHNlO1xuICBpZiAoIWNoZWNrUGFyYW1zLmFwcGx5KHZvaWQgMCwgYXJndW1lbnRzKSkgcmV0dXJuIGZhbHNlO1xuXG4gIHRyeSB7XG4gICAgLy8gR2V0IHRoZSB0cmFuc2l0aW9uIGJlemllciBjdXJ2ZVxuICAgIHZhciBiZXppZXJDdXJ2ZSA9IGdldEJlemllckN1cnZlKHRCQyk7IC8vIEdldCB0aGUgcHJvZ3Jlc3Mgb2YgZWFjaCBmcmFtZSBzdGF0ZVxuXG4gICAgdmFyIGZyYW1lU3RhdGVQcm9ncmVzcyA9IGdldEZyYW1lU3RhdGVQcm9ncmVzcyhiZXppZXJDdXJ2ZSwgZnJhbWVOdW0pOyAvLyBJZiB0aGUgcmVjdXJzaW9uIG1vZGUgaXMgbm90IGVuYWJsZWQgb3IgdGhlIHN0YXRlIHR5cGUgaXMgTnVtYmVyLCB0aGUgc2hhbGxvdyBzdGF0ZSBjYWxjdWxhdGlvbiBpcyBwZXJmb3JtZWQgZGlyZWN0bHkuXG5cbiAgICBpZiAoIWRlZXAgfHwgdHlwZW9mIGVuZFN0YXRlID09PSAnbnVtYmVyJykgcmV0dXJuIGdldFRyYW5zaXRpb25TdGF0ZShzdGFydFN0YXRlLCBlbmRTdGF0ZSwgZnJhbWVTdGF0ZVByb2dyZXNzKTtcbiAgICByZXR1cm4gcmVjdXJzaW9uVHJhbnNpdGlvblN0YXRlKHN0YXJ0U3RhdGUsIGVuZFN0YXRlLCBmcmFtZVN0YXRlUHJvZ3Jlc3MpO1xuICB9IGNhdGNoIChfdW51c2VkKSB7XG4gICAgY29uc29sZS53YXJuKCdUcmFuc2l0aW9uIHBhcmFtZXRlciBtYXkgYmUgYWJub3JtYWwhJyk7XG4gICAgcmV0dXJuIFtlbmRTdGF0ZV07XG4gIH1cbn1cbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gQ2hlY2sgaWYgdGhlIHBhcmFtZXRlcnMgYXJlIGxlZ2FsXHJcbiAqIEBwYXJhbSB7U3RyaW5nfSB0QkMgICAgICBOYW1lIG9mIHRyYW5zaXRpb24gYmV6aWVyIGN1cnZlXHJcbiAqIEBwYXJhbSB7QW55fSBzdGFydFN0YXRlICBUcmFuc2l0aW9uIHN0YXJ0IHN0YXRlXHJcbiAqIEBwYXJhbSB7QW55fSBlbmRTdGF0ZSAgICBUcmFuc2l0aW9uIGVuZCBzdGF0ZVxyXG4gKiBAcGFyYW0ge051bWJlcn0gZnJhbWVOdW0gTnVtYmVyIG9mIHRyYW5zaXRpb24gZnJhbWVzXHJcbiAqIEByZXR1cm4ge0Jvb2xlYW59IElzIHRoZSBwYXJhbWV0ZXIgbGVnYWxcclxuICovXG5cblxuZnVuY3Rpb24gY2hlY2tQYXJhbXModEJDKSB7XG4gIHZhciBzdGFydFN0YXRlID0gYXJndW1lbnRzLmxlbmd0aCA+IDEgJiYgYXJndW1lbnRzWzFdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMV0gOiBmYWxzZTtcbiAgdmFyIGVuZFN0YXRlID0gYXJndW1lbnRzLmxlbmd0aCA+IDIgJiYgYXJndW1lbnRzWzJdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbMl0gOiBmYWxzZTtcbiAgdmFyIGZyYW1lTnVtID0gYXJndW1lbnRzLmxlbmd0aCA+IDMgJiYgYXJndW1lbnRzWzNdICE9PSB1bmRlZmluZWQgPyBhcmd1bWVudHNbM10gOiAzMDtcblxuICBpZiAoIXRCQyB8fCBzdGFydFN0YXRlID09PSBmYWxzZSB8fCBlbmRTdGF0ZSA9PT0gZmFsc2UgfHwgIWZyYW1lTnVtKSB7XG4gICAgY29uc29sZS5lcnJvcigndHJhbnNpdGlvbjogTWlzc2luZyBQYXJhbWV0ZXJzIScpO1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIGlmICgoMCwgX3R5cGVvZjJbXCJkZWZhdWx0XCJdKShzdGFydFN0YXRlKSAhPT0gKDAsIF90eXBlb2YyW1wiZGVmYXVsdFwiXSkoZW5kU3RhdGUpKSB7XG4gICAgY29uc29sZS5lcnJvcigndHJhbnNpdGlvbjogSW5jb25zaXN0ZW50IFN0YXR1cyBUeXBlcyEnKTtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cblxuICB2YXIgc3RhdGVUeXBlID0gKDAsIF90eXBlb2YyW1wiZGVmYXVsdFwiXSkoZW5kU3RhdGUpO1xuXG4gIGlmIChzdGF0ZVR5cGUgPT09ICdzdHJpbmcnIHx8IHN0YXRlVHlwZSA9PT0gJ2Jvb2xlYW4nIHx8ICF0QkMubGVuZ3RoKSB7XG4gICAgY29uc29sZS5lcnJvcigndHJhbnNpdGlvbjogVW5zdXBwb3J0ZWQgRGF0YSBUeXBlIG9mIFN0YXRlIScpO1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIGlmICghX2N1cnZlc1tcImRlZmF1bHRcIl0uaGFzKHRCQykgJiYgISh0QkMgaW5zdGFuY2VvZiBBcnJheSkpIHtcbiAgICBjb25zb2xlLndhcm4oJ3RyYW5zaXRpb246IFRyYW5zaXRpb24gY3VydmUgbm90IGZvdW5kLCBkZWZhdWx0IGN1cnZlIHdpbGwgYmUgdXNlZCEnKTtcbiAgfVxuXG4gIHJldHVybiB0cnVlO1xufVxuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBHZXQgdGhlIHRyYW5zaXRpb24gYmV6aWVyIGN1cnZlXHJcbiAqIEBwYXJhbSB7U3RyaW5nfSB0QkMgTmFtZSBvZiB0cmFuc2l0aW9uIGJlemllciBjdXJ2ZVxyXG4gKiBAcmV0dXJuIHtBcnJheX0gQmV6aWVyIGN1cnZlIGRhdGFcclxuICovXG5cblxuZnVuY3Rpb24gZ2V0QmV6aWVyQ3VydmUodEJDKSB7XG4gIHZhciBiZXppZXJDdXJ2ZSA9ICcnO1xuXG4gIGlmIChfY3VydmVzW1wiZGVmYXVsdFwiXS5oYXModEJDKSkge1xuICAgIGJlemllckN1cnZlID0gX2N1cnZlc1tcImRlZmF1bHRcIl0uZ2V0KHRCQyk7XG4gIH0gZWxzZSBpZiAodEJDIGluc3RhbmNlb2YgQXJyYXkpIHtcbiAgICBiZXppZXJDdXJ2ZSA9IHRCQztcbiAgfSBlbHNlIHtcbiAgICBiZXppZXJDdXJ2ZSA9IF9jdXJ2ZXNbXCJkZWZhdWx0XCJdLmdldChkZWZhdWx0VHJhbnNpdGlvbkJDKTtcbiAgfVxuXG4gIHJldHVybiBiZXppZXJDdXJ2ZTtcbn1cbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gR2V0IHRoZSBwcm9ncmVzcyBvZiBlYWNoIGZyYW1lIHN0YXRlXHJcbiAqIEBwYXJhbSB7QXJyYXl9IGJlemllckN1cnZlIFRyYW5zaXRpb24gYmV6aWVyIGN1cnZlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSBmcmFtZU51bSAgIE51bWJlciBvZiB0cmFuc2l0aW9uIGZyYW1lc1xyXG4gKiBAcmV0dXJuIHtBcnJheX0gUHJvZ3Jlc3Mgb2YgZWFjaCBmcmFtZSBzdGF0ZVxyXG4gKi9cblxuXG5mdW5jdGlvbiBnZXRGcmFtZVN0YXRlUHJvZ3Jlc3MoYmV6aWVyQ3VydmUsIGZyYW1lTnVtKSB7XG4gIHZhciB0TWludXMgPSAxIC8gKGZyYW1lTnVtIC0gMSk7XG4gIHZhciB0U3RhdGUgPSBuZXcgQXJyYXkoZnJhbWVOdW0pLmZpbGwoMCkubWFwKGZ1bmN0aW9uICh0LCBpKSB7XG4gICAgcmV0dXJuIGkgKiB0TWludXM7XG4gIH0pO1xuICB2YXIgZnJhbWVTdGF0ZSA9IHRTdGF0ZS5tYXAoZnVuY3Rpb24gKHQpIHtcbiAgICByZXR1cm4gZ2V0RnJhbWVTdGF0ZUZyb21UKGJlemllckN1cnZlLCB0KTtcbiAgfSk7XG4gIHJldHVybiBmcmFtZVN0YXRlO1xufVxuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBHZXQgdGhlIHByb2dyZXNzIG9mIHRoZSBjb3JyZXNwb25kaW5nIGZyYW1lIGFjY29yZGluZyB0byB0XHJcbiAqIEBwYXJhbSB7QXJyYXl9IGJlemllckN1cnZlIFRyYW5zaXRpb24gYmV6aWVyIGN1cnZlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSB0ICAgICAgICAgIEN1cnJlbnQgZnJhbWUgdFxyXG4gKiBAcmV0dXJuIHtOdW1iZXJ9IFByb2dyZXNzIG9mIGN1cnJlbnQgZnJhbWVcclxuICovXG5cblxuZnVuY3Rpb24gZ2V0RnJhbWVTdGF0ZUZyb21UKGJlemllckN1cnZlLCB0KSB7XG4gIHZhciB0QmV6aWVyQ3VydmVQb2ludCA9IGdldEJlemllckN1cnZlUG9pbnRGcm9tVChiZXppZXJDdXJ2ZSwgdCk7XG4gIHZhciBiZXppZXJDdXJ2ZVBvaW50VCA9IGdldEJlemllckN1cnZlUG9pbnRURnJvbVJlVCh0QmV6aWVyQ3VydmVQb2ludCwgdCk7XG4gIHJldHVybiBnZXRCZXppZXJDdXJ2ZVRTdGF0ZSh0QmV6aWVyQ3VydmVQb2ludCwgYmV6aWVyQ3VydmVQb2ludFQpO1xufVxuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBHZXQgdGhlIGNvcnJlc3BvbmRpbmcgc3ViLWN1cnZlIGFjY29yZGluZyB0byB0XHJcbiAqIEBwYXJhbSB7QXJyYXl9IGJlemllckN1cnZlIFRyYW5zaXRpb24gYmV6aWVyIGN1cnZlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSB0ICAgICAgICAgIEN1cnJlbnQgZnJhbWUgdFxyXG4gKiBAcmV0dXJuIHtBcnJheX0gU3ViLWN1cnZlIG9mIHRcclxuICovXG5cblxuZnVuY3Rpb24gZ2V0QmV6aWVyQ3VydmVQb2ludEZyb21UKGJlemllckN1cnZlLCB0KSB7XG4gIHZhciBsYXN0SW5kZXggPSBiZXppZXJDdXJ2ZS5sZW5ndGggLSAxO1xuICB2YXIgYmVnaW4gPSAnJyxcbiAgICAgIGVuZCA9ICcnO1xuICBiZXppZXJDdXJ2ZS5maW5kSW5kZXgoZnVuY3Rpb24gKGl0ZW0sIGkpIHtcbiAgICBpZiAoaSA9PT0gbGFzdEluZGV4KSByZXR1cm47XG4gICAgYmVnaW4gPSBpdGVtO1xuICAgIGVuZCA9IGJlemllckN1cnZlW2kgKyAxXTtcbiAgICB2YXIgY3VycmVudE1haW5Qb2ludFggPSBiZWdpblswXVswXTtcbiAgICB2YXIgbmV4dE1haW5Qb2ludFggPSBlbmRbMF1bMF07XG4gICAgcmV0dXJuIHQgPj0gY3VycmVudE1haW5Qb2ludFggJiYgdCA8IG5leHRNYWluUG9pbnRYO1xuICB9KTtcbiAgdmFyIHAwID0gYmVnaW5bMF07XG4gIHZhciBwMSA9IGJlZ2luWzJdIHx8IGJlZ2luWzBdO1xuICB2YXIgcDIgPSBlbmRbMV0gfHwgZW5kWzBdO1xuICB2YXIgcDMgPSBlbmRbMF07XG4gIHJldHVybiBbcDAsIHAxLCBwMiwgcDNdO1xufVxuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBHZXQgbG9jYWwgdCBiYXNlZCBvbiB0IGFuZCBzdWItY3VydmVcclxuICogQHBhcmFtIHtBcnJheX0gYmV6aWVyQ3VydmUgU3ViLWN1cnZlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSB0ICAgICAgICAgIEN1cnJlbnQgZnJhbWUgdFxyXG4gKiBAcmV0dXJuIHtOdW1iZXJ9IGxvY2FsIHQgb2Ygc3ViLWN1cnZlXHJcbiAqL1xuXG5cbmZ1bmN0aW9uIGdldEJlemllckN1cnZlUG9pbnRURnJvbVJlVChiZXppZXJDdXJ2ZSwgdCkge1xuICB2YXIgcmVCZWdpblggPSBiZXppZXJDdXJ2ZVswXVswXTtcbiAgdmFyIHJlRW5kWCA9IGJlemllckN1cnZlWzNdWzBdO1xuICB2YXIgeE1pbnVzID0gcmVFbmRYIC0gcmVCZWdpblg7XG4gIHZhciB0TWludXMgPSB0IC0gcmVCZWdpblg7XG4gIHJldHVybiB0TWludXMgLyB4TWludXM7XG59XG4vKipcclxuICogQGRlc2NyaXB0aW9uIEdldCB0aGUgY3VydmUgcHJvZ3Jlc3Mgb2YgdFxyXG4gKiBAcGFyYW0ge0FycmF5fSBiZXppZXJDdXJ2ZSBTdWItY3VydmVcclxuICogQHBhcmFtIHtOdW1iZXJ9IHQgICAgICAgICAgQ3VycmVudCBmcmFtZSB0XHJcbiAqIEByZXR1cm4ge051bWJlcn0gUHJvZ3Jlc3Mgb2YgY3VycmVudCBmcmFtZVxyXG4gKi9cblxuXG5mdW5jdGlvbiBnZXRCZXppZXJDdXJ2ZVRTdGF0ZShfcmVmLCB0KSB7XG4gIHZhciBfcmVmMiA9ICgwLCBfc2xpY2VkVG9BcnJheTJbXCJkZWZhdWx0XCJdKShfcmVmLCA0KSxcbiAgICAgIF9yZWYyJCA9ICgwLCBfc2xpY2VkVG9BcnJheTJbXCJkZWZhdWx0XCJdKShfcmVmMlswXSwgMiksXG4gICAgICBwMCA9IF9yZWYyJFsxXSxcbiAgICAgIF9yZWYyJDIgPSAoMCwgX3NsaWNlZFRvQXJyYXkyW1wiZGVmYXVsdFwiXSkoX3JlZjJbMV0sIDIpLFxuICAgICAgcDEgPSBfcmVmMiQyWzFdLFxuICAgICAgX3JlZjIkMyA9ICgwLCBfc2xpY2VkVG9BcnJheTJbXCJkZWZhdWx0XCJdKShfcmVmMlsyXSwgMiksXG4gICAgICBwMiA9IF9yZWYyJDNbMV0sXG4gICAgICBfcmVmMiQ0ID0gKDAsIF9zbGljZWRUb0FycmF5MltcImRlZmF1bHRcIl0pKF9yZWYyWzNdLCAyKSxcbiAgICAgIHAzID0gX3JlZjIkNFsxXTtcblxuICB2YXIgcG93ID0gTWF0aC5wb3c7XG4gIHZhciB0TWludXMgPSAxIC0gdDtcbiAgdmFyIHJlc3VsdDEgPSBwMCAqIHBvdyh0TWludXMsIDMpO1xuICB2YXIgcmVzdWx0MiA9IDMgKiBwMSAqIHQgKiBwb3codE1pbnVzLCAyKTtcbiAgdmFyIHJlc3VsdDMgPSAzICogcDIgKiBwb3codCwgMikgKiB0TWludXM7XG4gIHZhciByZXN1bHQ0ID0gcDMgKiBwb3codCwgMyk7XG4gIHJldHVybiAxIC0gKHJlc3VsdDEgKyByZXN1bHQyICsgcmVzdWx0MyArIHJlc3VsdDQpO1xufVxuLyoqXHJcbiAqIEBkZXNjcmlwdGlvbiBHZXQgdHJhbnNpdGlvbiBzdGF0ZSBhY2NvcmRpbmcgdG8gZnJhbWUgcHJvZ3Jlc3NcclxuICogQHBhcmFtIHtBbnl9IHN0YXJ0U3RhdGUgICBUcmFuc2l0aW9uIHN0YXJ0IHN0YXRlXHJcbiAqIEBwYXJhbSB7QW55fSBlbmRTdGF0ZSAgICAgVHJhbnNpdGlvbiBlbmQgc3RhdGVcclxuICogQHBhcmFtIHtBcnJheX0gZnJhbWVTdGF0ZSBGcmFtZSBzdGF0ZSBwcm9ncmVzc1xyXG4gKiBAcmV0dXJuIHtBcnJheX0gVHJhbnNpdGlvbiBmcmFtZSBzdGF0ZVxyXG4gKi9cblxuXG5mdW5jdGlvbiBnZXRUcmFuc2l0aW9uU3RhdGUoYmVnaW4sIGVuZCwgZnJhbWVTdGF0ZSkge1xuICB2YXIgc3RhdGVUeXBlID0gJ29iamVjdCc7XG4gIGlmICh0eXBlb2YgYmVnaW4gPT09ICdudW1iZXInKSBzdGF0ZVR5cGUgPSAnbnVtYmVyJztcbiAgaWYgKGJlZ2luIGluc3RhbmNlb2YgQXJyYXkpIHN0YXRlVHlwZSA9ICdhcnJheSc7XG4gIGlmIChzdGF0ZVR5cGUgPT09ICdudW1iZXInKSByZXR1cm4gZ2V0TnVtYmVyVHJhbnNpdGlvblN0YXRlKGJlZ2luLCBlbmQsIGZyYW1lU3RhdGUpO1xuICBpZiAoc3RhdGVUeXBlID09PSAnYXJyYXknKSByZXR1cm4gZ2V0QXJyYXlUcmFuc2l0aW9uU3RhdGUoYmVnaW4sIGVuZCwgZnJhbWVTdGF0ZSk7XG4gIGlmIChzdGF0ZVR5cGUgPT09ICdvYmplY3QnKSByZXR1cm4gZ2V0T2JqZWN0VHJhbnNpdGlvblN0YXRlKGJlZ2luLCBlbmQsIGZyYW1lU3RhdGUpO1xuICByZXR1cm4gZnJhbWVTdGF0ZS5tYXAoZnVuY3Rpb24gKHQpIHtcbiAgICByZXR1cm4gZW5kO1xuICB9KTtcbn1cbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gR2V0IHRoZSB0cmFuc2l0aW9uIGRhdGEgb2YgdGhlIG51bWJlciB0eXBlXHJcbiAqIEBwYXJhbSB7TnVtYmVyfSBzdGFydFN0YXRlIFRyYW5zaXRpb24gc3RhcnQgc3RhdGVcclxuICogQHBhcmFtIHtOdW1iZXJ9IGVuZFN0YXRlICAgVHJhbnNpdGlvbiBlbmQgc3RhdGVcclxuICogQHBhcmFtIHtBcnJheX0gZnJhbWVTdGF0ZSAgRnJhbWUgc3RhdGUgcHJvZ3Jlc3NcclxuICogQHJldHVybiB7QXJyYXl9IFRyYW5zaXRpb24gZnJhbWUgc3RhdGVcclxuICovXG5cblxuZnVuY3Rpb24gZ2V0TnVtYmVyVHJhbnNpdGlvblN0YXRlKGJlZ2luLCBlbmQsIGZyYW1lU3RhdGUpIHtcbiAgdmFyIG1pbnVzID0gZW5kIC0gYmVnaW47XG4gIHJldHVybiBmcmFtZVN0YXRlLm1hcChmdW5jdGlvbiAocykge1xuICAgIHJldHVybiBiZWdpbiArIG1pbnVzICogcztcbiAgfSk7XG59XG4vKipcclxuICogQGRlc2NyaXB0aW9uIEdldCB0aGUgdHJhbnNpdGlvbiBkYXRhIG9mIHRoZSBhcnJheSB0eXBlXHJcbiAqIEBwYXJhbSB7QXJyYXl9IHN0YXJ0U3RhdGUgVHJhbnNpdGlvbiBzdGFydCBzdGF0ZVxyXG4gKiBAcGFyYW0ge0FycmF5fSBlbmRTdGF0ZSAgIFRyYW5zaXRpb24gZW5kIHN0YXRlXHJcbiAqIEBwYXJhbSB7QXJyYXl9IGZyYW1lU3RhdGUgRnJhbWUgc3RhdGUgcHJvZ3Jlc3NcclxuICogQHJldHVybiB7QXJyYXl9IFRyYW5zaXRpb24gZnJhbWUgc3RhdGVcclxuICovXG5cblxuZnVuY3Rpb24gZ2V0QXJyYXlUcmFuc2l0aW9uU3RhdGUoYmVnaW4sIGVuZCwgZnJhbWVTdGF0ZSkge1xuICB2YXIgbWludXMgPSBlbmQubWFwKGZ1bmN0aW9uICh2LCBpKSB7XG4gICAgaWYgKHR5cGVvZiB2ICE9PSAnbnVtYmVyJykgcmV0dXJuIGZhbHNlO1xuICAgIHJldHVybiB2IC0gYmVnaW5baV07XG4gIH0pO1xuICByZXR1cm4gZnJhbWVTdGF0ZS5tYXAoZnVuY3Rpb24gKHMpIHtcbiAgICByZXR1cm4gbWludXMubWFwKGZ1bmN0aW9uICh2LCBpKSB7XG4gICAgICBpZiAodiA9PT0gZmFsc2UpIHJldHVybiBlbmRbaV07XG4gICAgICByZXR1cm4gYmVnaW5baV0gKyB2ICogcztcbiAgICB9KTtcbiAgfSk7XG59XG4vKipcclxuICogQGRlc2NyaXB0aW9uIEdldCB0aGUgdHJhbnNpdGlvbiBkYXRhIG9mIHRoZSBvYmplY3QgdHlwZVxyXG4gKiBAcGFyYW0ge09iamVjdH0gc3RhcnRTdGF0ZSBUcmFuc2l0aW9uIHN0YXJ0IHN0YXRlXHJcbiAqIEBwYXJhbSB7T2JqZWN0fSBlbmRTdGF0ZSAgIFRyYW5zaXRpb24gZW5kIHN0YXRlXHJcbiAqIEBwYXJhbSB7QXJyYXl9IGZyYW1lU3RhdGUgIEZyYW1lIHN0YXRlIHByb2dyZXNzXHJcbiAqIEByZXR1cm4ge0FycmF5fSBUcmFuc2l0aW9uIGZyYW1lIHN0YXRlXHJcbiAqL1xuXG5cbmZ1bmN0aW9uIGdldE9iamVjdFRyYW5zaXRpb25TdGF0ZShiZWdpbiwgZW5kLCBmcmFtZVN0YXRlKSB7XG4gIHZhciBrZXlzID0gT2JqZWN0LmtleXMoZW5kKTtcbiAgdmFyIGJlZ2luVmFsdWUgPSBrZXlzLm1hcChmdW5jdGlvbiAoaykge1xuICAgIHJldHVybiBiZWdpbltrXTtcbiAgfSk7XG4gIHZhciBlbmRWYWx1ZSA9IGtleXMubWFwKGZ1bmN0aW9uIChrKSB7XG4gICAgcmV0dXJuIGVuZFtrXTtcbiAgfSk7XG4gIHZhciBhcnJheVN0YXRlID0gZ2V0QXJyYXlUcmFuc2l0aW9uU3RhdGUoYmVnaW5WYWx1ZSwgZW5kVmFsdWUsIGZyYW1lU3RhdGUpO1xuICByZXR1cm4gYXJyYXlTdGF0ZS5tYXAoZnVuY3Rpb24gKGl0ZW0pIHtcbiAgICB2YXIgZnJhbWVEYXRhID0ge307XG4gICAgaXRlbS5mb3JFYWNoKGZ1bmN0aW9uICh2LCBpKSB7XG4gICAgICByZXR1cm4gZnJhbWVEYXRhW2tleXNbaV1dID0gdjtcbiAgICB9KTtcbiAgICByZXR1cm4gZnJhbWVEYXRhO1xuICB9KTtcbn1cbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gR2V0IHRoZSB0cmFuc2l0aW9uIHN0YXRlIGRhdGEgYnkgcmVjdXJzaW9uXHJcbiAqIEBwYXJhbSB7QXJyYXl8T2JqZWN0fSBzdGFydFN0YXRlIFRyYW5zaXRpb24gc3RhcnQgc3RhdGVcclxuICogQHBhcmFtIHtBcnJheXxPYmplY3R9IGVuZFN0YXRlICAgVHJhbnNpdGlvbiBlbmQgc3RhdGVcclxuICogQHBhcmFtIHtBcnJheX0gZnJhbWVTdGF0ZSAgICAgICAgRnJhbWUgc3RhdGUgcHJvZ3Jlc3NcclxuICogQHJldHVybiB7QXJyYXl9IFRyYW5zaXRpb24gZnJhbWUgc3RhdGVcclxuICovXG5cblxuZnVuY3Rpb24gcmVjdXJzaW9uVHJhbnNpdGlvblN0YXRlKGJlZ2luLCBlbmQsIGZyYW1lU3RhdGUpIHtcbiAgdmFyIHN0YXRlID0gZ2V0VHJhbnNpdGlvblN0YXRlKGJlZ2luLCBlbmQsIGZyYW1lU3RhdGUpO1xuXG4gIHZhciBfbG9vcCA9IGZ1bmN0aW9uIF9sb29wKGtleSkge1xuICAgIHZhciBiVGVtcCA9IGJlZ2luW2tleV07XG4gICAgdmFyIGVUZW1wID0gZW5kW2tleV07XG4gICAgaWYgKCgwLCBfdHlwZW9mMltcImRlZmF1bHRcIl0pKGVUZW1wKSAhPT0gJ29iamVjdCcpIHJldHVybiBcImNvbnRpbnVlXCI7XG4gICAgdmFyIGRhdGEgPSByZWN1cnNpb25UcmFuc2l0aW9uU3RhdGUoYlRlbXAsIGVUZW1wLCBmcmFtZVN0YXRlKTtcbiAgICBzdGF0ZS5mb3JFYWNoKGZ1bmN0aW9uIChmcywgaSkge1xuICAgICAgcmV0dXJuIGZzW2tleV0gPSBkYXRhW2ldO1xuICAgIH0pO1xuICB9O1xuXG4gIGZvciAodmFyIGtleSBpbiBlbmQpIHtcbiAgICB2YXIgX3JldCA9IF9sb29wKGtleSk7XG5cbiAgICBpZiAoX3JldCA9PT0gXCJjb250aW51ZVwiKSBjb250aW51ZTtcbiAgfVxuXG4gIHJldHVybiBzdGF0ZTtcbn1cbi8qKlxyXG4gKiBAZGVzY3JpcHRpb24gSW5qZWN0IG5ldyBjdXJ2ZSBpbnRvIGN1cnZlcyBhcyBjb25maWdcclxuICogQHBhcmFtIHtBbnl9IGtleSAgICAgVGhlIGtleSBvZiBjdXJ2ZVxyXG4gKiBAcGFyYW0ge0FycmF5fSBjdXJ2ZSBCZXppZXIgY3VydmUgZGF0YVxyXG4gKiBAcmV0dXJuIHtVbmRlZmluZWR9IE5vIHJldHVyblxyXG4gKi9cblxuXG5mdW5jdGlvbiBpbmplY3ROZXdDdXJ2ZShrZXksIGN1cnZlKSB7XG4gIGlmICgha2V5IHx8ICFjdXJ2ZSkge1xuICAgIGNvbnNvbGUuZXJyb3IoJ0luamVjdE5ld0N1cnZlIE1pc3NpbmcgUGFyYW1ldGVycyEnKTtcbiAgICByZXR1cm47XG4gIH1cblxuICBfY3VydmVzW1wiZGVmYXVsdFwiXS5zZXQoa2V5LCBjdXJ2ZSk7XG59XG5cbnZhciBfZGVmYXVsdCA9IHRyYW5zaXRpb247XG5leHBvcnRzW1wiZGVmYXVsdFwiXSA9IF9kZWZhdWx0OyIsImZ1bmN0aW9uIF9hcnJheVdpdGhIb2xlcyhhcnIpIHtcbiAgaWYgKEFycmF5LmlzQXJyYXkoYXJyKSkgcmV0dXJuIGFycjtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBfYXJyYXlXaXRoSG9sZXM7IiwiZnVuY3Rpb24gX2ludGVyb3BSZXF1aXJlRGVmYXVsdChvYmopIHtcbiAgcmV0dXJuIG9iaiAmJiBvYmouX19lc01vZHVsZSA/IG9iaiA6IHtcbiAgICBcImRlZmF1bHRcIjogb2JqXG4gIH07XG59XG5cbm1vZHVsZS5leHBvcnRzID0gX2ludGVyb3BSZXF1aXJlRGVmYXVsdDsiLCJmdW5jdGlvbiBfaXRlcmFibGVUb0FycmF5TGltaXQoYXJyLCBpKSB7XG4gIHZhciBfYXJyID0gW107XG4gIHZhciBfbiA9IHRydWU7XG4gIHZhciBfZCA9IGZhbHNlO1xuICB2YXIgX2UgPSB1bmRlZmluZWQ7XG5cbiAgdHJ5IHtcbiAgICBmb3IgKHZhciBfaSA9IGFycltTeW1ib2wuaXRlcmF0b3JdKCksIF9zOyAhKF9uID0gKF9zID0gX2kubmV4dCgpKS5kb25lKTsgX24gPSB0cnVlKSB7XG4gICAgICBfYXJyLnB1c2goX3MudmFsdWUpO1xuXG4gICAgICBpZiAoaSAmJiBfYXJyLmxlbmd0aCA9PT0gaSkgYnJlYWs7XG4gICAgfVxuICB9IGNhdGNoIChlcnIpIHtcbiAgICBfZCA9IHRydWU7XG4gICAgX2UgPSBlcnI7XG4gIH0gZmluYWxseSB7XG4gICAgdHJ5IHtcbiAgICAgIGlmICghX24gJiYgX2lbXCJyZXR1cm5cIl0gIT0gbnVsbCkgX2lbXCJyZXR1cm5cIl0oKTtcbiAgICB9IGZpbmFsbHkge1xuICAgICAgaWYgKF9kKSB0aHJvdyBfZTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gX2Fycjtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBfaXRlcmFibGVUb0FycmF5TGltaXQ7IiwiZnVuY3Rpb24gX25vbkl0ZXJhYmxlUmVzdCgpIHtcbiAgdGhyb3cgbmV3IFR5cGVFcnJvcihcIkludmFsaWQgYXR0ZW1wdCB0byBkZXN0cnVjdHVyZSBub24taXRlcmFibGUgaW5zdGFuY2VcIik7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gX25vbkl0ZXJhYmxlUmVzdDsiLCJ2YXIgYXJyYXlXaXRoSG9sZXMgPSByZXF1aXJlKFwiLi9hcnJheVdpdGhIb2xlc1wiKTtcblxudmFyIGl0ZXJhYmxlVG9BcnJheUxpbWl0ID0gcmVxdWlyZShcIi4vaXRlcmFibGVUb0FycmF5TGltaXRcIik7XG5cbnZhciBub25JdGVyYWJsZVJlc3QgPSByZXF1aXJlKFwiLi9ub25JdGVyYWJsZVJlc3RcIik7XG5cbmZ1bmN0aW9uIF9zbGljZWRUb0FycmF5KGFyciwgaSkge1xuICByZXR1cm4gYXJyYXlXaXRoSG9sZXMoYXJyKSB8fCBpdGVyYWJsZVRvQXJyYXlMaW1pdChhcnIsIGkpIHx8IG5vbkl0ZXJhYmxlUmVzdCgpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IF9zbGljZWRUb0FycmF5OyIsImZ1bmN0aW9uIF90eXBlb2YyKG9iaikgeyBpZiAodHlwZW9mIFN5bWJvbCA9PT0gXCJmdW5jdGlvblwiICYmIHR5cGVvZiBTeW1ib2wuaXRlcmF0b3IgPT09IFwic3ltYm9sXCIpIHsgX3R5cGVvZjIgPSBmdW5jdGlvbiBfdHlwZW9mMihvYmopIHsgcmV0dXJuIHR5cGVvZiBvYmo7IH07IH0gZWxzZSB7IF90eXBlb2YyID0gZnVuY3Rpb24gX3R5cGVvZjIob2JqKSB7IHJldHVybiBvYmogJiYgdHlwZW9mIFN5bWJvbCA9PT0gXCJmdW5jdGlvblwiICYmIG9iai5jb25zdHJ1Y3RvciA9PT0gU3ltYm9sICYmIG9iaiAhPT0gU3ltYm9sLnByb3RvdHlwZSA/IFwic3ltYm9sXCIgOiB0eXBlb2Ygb2JqOyB9OyB9IHJldHVybiBfdHlwZW9mMihvYmopOyB9XG5cbmZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7XG4gIGlmICh0eXBlb2YgU3ltYm9sID09PSBcImZ1bmN0aW9uXCIgJiYgX3R5cGVvZjIoU3ltYm9sLml0ZXJhdG9yKSA9PT0gXCJzeW1ib2xcIikge1xuICAgIG1vZHVsZS5leHBvcnRzID0gX3R5cGVvZiA9IGZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7XG4gICAgICByZXR1cm4gX3R5cGVvZjIob2JqKTtcbiAgICB9O1xuICB9IGVsc2Uge1xuICAgIG1vZHVsZS5leHBvcnRzID0gX3R5cGVvZiA9IGZ1bmN0aW9uIF90eXBlb2Yob2JqKSB7XG4gICAgICByZXR1cm4gb2JqICYmIHR5cGVvZiBTeW1ib2wgPT09IFwiZnVuY3Rpb25cIiAmJiBvYmouY29uc3RydWN0b3IgPT09IFN5bWJvbCAmJiBvYmogIT09IFN5bWJvbC5wcm90b3R5cGUgPyBcInN5bWJvbFwiIDogX3R5cGVvZjIob2JqKTtcbiAgICB9O1xuICB9XG5cbiAgcmV0dXJuIF90eXBlb2Yob2JqKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBfdHlwZW9mOyJdfQ==
