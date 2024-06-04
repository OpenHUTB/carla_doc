const { expect } = require('chai')

const {
  deepClone,
  eliminateBlur,
  getTwoPointDistance
} = require('../../src/plugin/util')

describe('deepClone', () => {
  it('deepClone()', () => {
    expect(deepClone()).to.be.undefined
  })

  it('deepClone(false)', () => {
    expect(deepClone(false)).to.be.false
  })

  it('deepClone({ a: 1, b: 2, c: 3 })', () => {
    expect(deepClone({ a: 1, b: 2, c: 3 })).to.deep.equal({ a: 1, b: 2, c: 3 })
  })

  it('deepClone([1, 2, 3])', () => {
    expect(deepClone([1, 2, 3])).to.ordered.deep.equal([1, 2, 3])
  })

  it('deepClone({ a: 1, b: [1, 2, 3], c: { d: 4 } })', () => {
    expect(deepClone({
      a: 1,
      b: [1, 2, 3],
      c: { d: 4 }
    })).to.ordered.deep.equal({
      a: 1,
      b: [1, 2, 3],
      c: { d: 4 }
    })
  })

  it('deepClone({ a: 1, b: 2, c: 3 }, true)', () => {
    expect(deepClone({ a: 1, b: 2, c: 3 }, true)).to.deep.equal({ a: 1, b: 2, c: 3 })
  })

  it('deepClone([1, 2, 3], true)', () => {
    expect(deepClone([1, 2, 3], true)).to.ordered.deep.equal([1, 2, 3])
  })

  it('deepClone({ a: 1, b: [1, 2, 3], c: { d: 4 } }, true)', () => {
    expect(deepClone({
      a: 1,
      b: [1, 2, 3],
      c: { d: 4 }
    }, true)).to.ordered.deep.equal({
      a: 1,
      b: [1, 2, 3],
      c: { d: 4 }
    })
  })
})

describe('eliminateBlur', () => {
  const points = [
    [1, 1],
    [2, 2],
    [3, 3]
  ]

  it('eliminateBlur', () => {
    expect(eliminateBlur(points)).to.ordered.deep.equal([
      [1.5, 1.5],
      [2.5, 2.5],
      [3.5, 3.5]
    ])
  })
})

describe('getTwoPointDistance', () => {
  it('getTwoPointDistance', () => {
    expect(getTwoPointDistance([0, 0], [1, 0])).to.be.equal(1)
  })
})
