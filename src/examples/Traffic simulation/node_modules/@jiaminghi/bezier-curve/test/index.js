const { expect } = require('chai')

const {
  bezierCurveToPolyline,
  getBezierCurveLength,
  polylineToBezierCurve
} = require('../src/index')

const bezierCurve = [
  [20, 20],
  [
    [100, 20],[100, 80],[180,80]
  ]
]

const polylinePrecision5 = bezierCurveToPolyline(bezierCurve)
const polylinePrecision10 = bezierCurveToPolyline(bezierCurve, 10)

function findNaNInArray (arr) {
  return arr.findIndex(n => !Number.isFinite(n)) !== -1
}

describe('bezierCurveToPolyline', () => {
  it('bezierCurveToPolyline()', () => {
    expect(bezierCurveToPolyline()).to.be.false
  })

  it('bezierCurveToPolyline({})', () => {
    expect(bezierCurveToPolyline({})).to.be.false
  })

  it(`bezierCurveToPolyline([], '1')`, () => {
    expect(bezierCurveToPolyline([], '1')).to.be.false
  })

  it('Return value is an array (Precision = 5)', () => {
    expect(polylinePrecision5).to.be.an('array')
  })

  it('Return value is an array (Precision = 10)', () => {
    expect(polylinePrecision10).to.be.an('array')
  })

  it('Return value length is at least 2 (Precision = 5)', () => {
    expect(polylinePrecision5).to.lengthOf.at.least(2)
  })

  it('Return value length is at least 2 (Precision = 10)', () => {
    expect(polylinePrecision10).to.lengthOf.at.least(2)
  })

  it('Return value element is an array (Precision = 5)', () => {
    expect(polylinePrecision5).to.satisfy(line => {
      return !line.find(item => !(item instanceof Array))
    })
  })

  it('Return value element is an array (Precision = 10)', () => {
    expect(polylinePrecision10).to.satisfy(line => {
      return !line.find(item => !(item instanceof Array))
    })
  })

  it('Return value element array length is 2 (Precision = 5)', () => {
    expect(polylinePrecision5).to.satisfy(line => {
      return !line.find(item => item.length !== 2)
    })
  })

  it('Return value element array length is 2 (Precision = 10)', () => {
    expect(polylinePrecision10).to.satisfy(line => {
      return !line.find(item => item.length !== 2)
    })
  })

  it('The value in the array of return value elements is a number (Precision = 5)', () => {
    expect(polylinePrecision5).to.satisfy(line => {
      return !line.find(item => item.findIndex(n => !Number.isFinite(n)) !== -1)
    })
  })

  it('The value in the array of return value elements is a number (Precision = 10)', () => {
    expect(polylinePrecision10).to.satisfy(line => {
      return !line.find(item => item.findIndex(n => !Number.isFinite(n)) !== -1)
    })
  })
})

describe('getBezierCurveLength', () => {
  it('getBezierCurveLength()', () => {
    expect(getBezierCurveLength()).to.be.false
  })

  it('getBezierCurveLength({})', () => {
    expect(getBezierCurveLength({})).to.be.false
  })

  it(`getBezierCurveLength([], '1')`, () => {
    expect(getBezierCurveLength([], '1')).to.be.false
  })

  it(`getBezierCurveLength(bezierCurve)`, () => {
    expect(getBezierCurveLength(bezierCurve)).to.be.finite.above(0)
  })

  it(`getBezierCurveLength(bezierCurve, 10)`, () => {
    expect(getBezierCurveLength(bezierCurve, 10)).to.be.finite.above(0)
  })
})

describe('polylineToBezierCurve', () => {
  it('polylineToBezierCurve()', () => {
    expect(polylineToBezierCurve()).to.be.false
  })

  it('polylineToBezierCurve({})', () => {
    expect(polylineToBezierCurve({})).to.be.false
  })

  it('Return value is an array (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.be.an('array')
  })

  it('Return value is an array (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.be.an('array')
  })

  it('Return value length is at least 2 (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.lengthOf.at.least(2)
  })

  it('Return value length is at least 2 (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.lengthOf.at.least(2)
  })

  it('Return value element is an array (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.satisfy(line => {
      return !line.find(item => !(item instanceof Array))
    })
  })

  it('Return value element is an array (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.satisfy(line => {
      return !line.find(item => !(item instanceof Array))
    })
  })

  it('Return value first element length is 2 (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.satisfy(line => {
      return line[0].length === 2
    })
  })

  it('Return value first element length is 2 (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.satisfy(line => {
      return line[0].length === 2
    })
  })

  it('Return value element length is 3 except first (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.satisfy(line => {
      return !line.find((item, i) => {
        if (i === 0) return false

        return item.length !== 3
      })
    })
  })

  it('Return value element length is 3 except first (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.satisfy(line => {
      return !line.find((item, i) => {
        if (i === 0) return false

        return item.length !== 3
      })
    })
  })

  it('The value in the array of return value elements is a number (polylinePrecision5)', () => {
    expect(polylineToBezierCurve(polylinePrecision5)).to.satisfy(line => {
      return !line.find((item, i) => {
        if (i === 0) {
          return findNaNInArray(item)
        } else {
          return item.find(itemElement => findNaNInArray(itemElement))
        }
      })
    })
  })

  it('The value in the array of return value elements is a number (polylinePrecision10)', () => {
    expect(polylineToBezierCurve(polylinePrecision10)).to.satisfy(line => {
      return !line.find((item, i) => {
        if (i === 0) {
          return findNaNInArray(item)
        } else {
          return item.find(itemElement => findNaNInArray(itemElement))
        }
      })
    })
  })
})