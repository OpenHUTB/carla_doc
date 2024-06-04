import type { RgbValue, RgbaValue } from './types/index';
/**
 * @description Test if a hex color
 * @param {string} color color string
 * @return {boolean} Test result
 */
export declare function isHex(color: string): boolean;
/**
 * @description Test if a rgb color
 * @param {string} color color string
 * @return {boolean} Test result
 */
export declare function isRgb(color: string): boolean;
/**
 * @description Test if a rgba color
 * @param {string} color color string
 * @return {boolean} Test result
 */
export declare function isRgba(color: string): boolean;
/**
 * @description Test if a rgb or rgba color
 * @param {string} color color string
 * @return {boolean} Test result
 */
export declare function isRgbOrRgba(color: string): boolean;
/**
 * @description Get the Rgb value of the color
 * @param {string} color Hex|Rgb|Rgba color or color keyword
 * @return {RgbValue} Rgb value of the color
 */
export declare function getRgbValue(color: string): RgbValue;
/**
 * @description Get the opacity of color
 * @param {string} color Hex|Rgb|Rgba color or color keyword
 * @return {number} Color opacity
 */
export declare function getOpacity(color: string): number;
/**
 * @description Get the Rgba value of the color
 * @param {string} color Hex|Rgb|Rgba color or color keyword
 * @return {RgbaValue} Rgba value of the color
 */
export declare function getRgbaValue(color: string): RgbaValue;
/**
 * @description Convert color to Rgb|Rgba color
 * @param {string} color   Hex|Rgb|Rgba color or color keyword
 * @param {number} opacity The opacity of color
 * @return {string} Rgb|Rgba color
 */
export declare function toRgb(color: string, opacity?: number): string;
/**
 * @description Convert color to Hex color
 * @param {string} color Hex|Rgb|Rgba color or color keyword
 * @return {string} Hex color
 */
export declare function toHex(color: string): string;
/**
 * @description Get Color from Rgb|Rgba value
 * @param {RgbValue|RgbaValue} value Rgb|Rgba color value
 * @return {string} Rgb|Rgba color
 */
export declare function getColorFromRgbValue(value: RgbValue | RgbaValue): string;
/**
 * @description Deepen color
 * @param {string} color   Hex|Rgb|Rgba color or color keyword
 * @param {number} percent of Deepen (1-100)
 * @return {string} Rgba color
 */
export declare function darken(color: string, percent?: number): string;
/**
 * @description Brighten color
 * @param {string} color   Hex|Rgb|Rgba color or color keyword
 * @param {number} percent of brighten (1-100)
 * @return {string} Rgba color
 */
export declare function lighten(color: string, percent?: number): string;
/**
 * @description Adjust color opacity
 * @param {string} color   Hex|Rgb|Rgba color or color keyword
 * @param {number} percent of opacity
 * @return {string} Rgba color
 */
export declare function fade(color: string, percent?: number): string;
