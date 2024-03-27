"use strict";

var _interopRequireDefault = require("@babel/runtime/helpers/interopRequireDefault");

Object.defineProperty(exports, "__esModule", {
  value: true
});
Object.defineProperty(exports, "CRender", {
  enumerable: true,
  get: function get() {
    return _crender["default"];
  }
});
Object.defineProperty(exports, "extendNewGraph", {
  enumerable: true,
  get: function get() {
    return _graphs.extendNewGraph;
  }
});
exports["default"] = void 0;

var _crender = _interopRequireDefault(require("./class/crender.class"));

var _graphs = require("./config/graphs");

var _default = _crender["default"];
exports["default"] = _default;