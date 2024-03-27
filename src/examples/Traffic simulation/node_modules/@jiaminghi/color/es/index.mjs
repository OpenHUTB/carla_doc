import { ColorKeywords as s } from "./config/keywords.mjs";
function i(r) {
  return typeof r != "string" ? !1 : (r = r.toLowerCase(), /^#([0-9a-fA-f]{3}|[0-9a-fA-f]{6})$/.test(r));
}
function C(r) {
  return typeof r != "string" ? !1 : (r = r.toLowerCase(), /^(rgb\(|RGB\()/.test(r));
}
function b(r) {
  return typeof r != "string" ? !1 : (r = r.toLowerCase(), /^(rgba|RGBA)/.test(r));
}
function l(r) {
  return /^(rgb|rgba|RGB|RGBA)/.test(r);
}
function p(r) {
  return s[r];
}
function g(r) {
  if (i(r) || l(r))
    return r;
  const t = p(r);
  if (!t)
    throw new Error(`Color: Invalid Input of ${r}`);
  return t;
}
function m(r) {
  r = r.replace("#", ""), r.length === 3 && (r = Array.from(r).map((e) => e + e).join(""));
  const t = r.split("");
  return new Array(3).fill(0).map((e, n) => parseInt(`0x${t[n * 2]}${t[n * 2 + 1]}`));
}
function R(r) {
  return r.replace(/rgb\(|rgba\(|\)/g, "").split(",").slice(0, 3).map((t) => parseInt(t));
}
function a(r) {
  const e = g(r).toLowerCase();
  return i(e) ? m(e) : R(e);
}
function c(r) {
  const t = g(r);
  return b(t) ? Number(
    t.toLowerCase().split(",").slice(-1)[0].replace(/[)|\s]/g, "")
  ) : 1;
}
function f(r) {
  const t = a(r);
  return t && [...t, c(r)];
}
function V(r, t) {
  const e = a(r);
  return typeof t == "number" ? `rgba(${e.join(",")},${t})` : `rgb(${e.join(",")})`;
}
function y(r) {
  if (i(r))
    return r;
  const t = a(r), e = (n) => Number(n).toString(16).padStart(2, "0");
  return `#${t.map(e).join("")}`;
}
function u(r) {
  if (!Array.isArray(r))
    throw new Error(`getColorFromRgbValue: ${r} is not an array`);
  const { length: t } = r;
  if (t !== 3 && t !== 4)
    throw new Error("getColorFromRgbValue: value length should be 3 or 4");
  return (t === 3 ? "rgb(" : "rgba(") + r.join(",") + ")";
}
function d(r, t = 0) {
  let e = f(r);
  return e = e.map((n, o) => o === 3 ? n : n - Math.ceil(2.55 * t)).map((n) => n < 0 ? 0 : n), u(e);
}
function h(r, t = 0) {
  let e = f(r);
  return e = e.map((n, o) => o === 3 ? n : n + Math.ceil(2.55 * t)).map((n) => n > 255 ? 255 : n), u(e);
}
function $(r, t = 100) {
  const e = a(r);
  return u([...e, t / 100]);
}
export {
  d as darken,
  $ as fade,
  u as getColorFromRgbValue,
  c as getOpacity,
  a as getRgbValue,
  f as getRgbaValue,
  i as isHex,
  C as isRgb,
  l as isRgbOrRgba,
  b as isRgba,
  h as lighten,
  y as toHex,
  V as toRgb
};
