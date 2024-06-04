const { expect } = require('chai')

const {
  transition,
  injectNewCurve
} = require('../src/index')

const testCurve = [[[0.000, 1.000]],[[0.151, 0.396], [0.019, 0.604], [0.330, 0.056]],[[1.000, 0.000]]]

function findNaNInArray (arr) {
  return arr.findIndex(n => !Number.isFinite(n)) !== -1
}

describe('transition-Number', () => {
  const startState = 0
  const endState = 100
  const frame = 30

  const linearResult = transition('linear', startState, endState, frame)
  const testCurveResult = transition(testCurve, startState, endState, frame)

  it('transition()', () => {
    expect(transition()).to.be.false
  })

  it('Input an invalid tBC will automatically apply the default tBC', () => {
    expect(transition('invalid', startState, endState, frame)).to.deep.ordered.equal(linearResult)
  })

  it('Transition result must be an array (linearResult)', () => {
    expect(linearResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result must be an array (testCurveResult)', () => {
    expect(testCurveResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result first element must equal startstate (linearResult)', () => {
    expect(linearResult).to.property('0', startState)
  })

  it('Transition result first element must equal startstate (testCurveResult)', () => {
    expect(testCurveResult).to.property('0', startState)
  })

  it('Transition result last element must equal endState (linearResult)', () => {
    expect(linearResult).to.property(frame - 1, endState)
  })

  it('Transition result last element must equal endState (testCurveResult)', () => {
    expect(testCurveResult).to.property(frame - 1, endState)
  })

  it('Transition result elements should all be number (linearResult)', () => {
    expect(linearResult).satisfy(states => {
      return !findNaNInArray(states)
    })
  })

  it('Transition result elements should all be number (testCurveResult)', () => {
    expect(testCurveResult).satisfy(states => {
      return !findNaNInArray(states)
    })
  })
})

describe('transition-Array', () => {
  const startState = [0, 0]
  const endState = [100, 100]
  const frame = 30

  const linearResult = transition('linear', startState, endState, frame)
  const testCurveResult = transition(testCurve, startState, endState, frame)

  it('Transition result must be an array (linearResult)', () => {
    expect(linearResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result must be an array (testCurveResult)', () => {
    expect(testCurveResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result first element must equal startstate (linearResult)', () => {
    expect(linearResult).to.deep.property('0', startState)
  })

  it('Transition result first element must equal startstate (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property('0', startState)
  })

  it('Transition result last element must equal endState (linearResult)', () => {
    expect(linearResult).to.deep.property(frame - 1, endState)
  })

  it('Transition result last element must equal endState (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property(frame - 1, endState)
  })

  it('Transition result elements should all be number (linearResult)', () => {
    expect(linearResult).satisfy(states => {
      return !states.find(item => findNaNInArray(item))
    })
  })

  it('Transition result elements should all be number (testCurveResult)', () => {
    expect(testCurveResult).satisfy(states => {
      return !states.find(item => findNaNInArray(item))
    })
  })
})

describe('transition-Object', () => {
  const startState = { x: 0, y: 0, name: 'circle'}
  const endState = { x: 100, y: 100, name: 'circle'}
  const frame = 30

  const linearResult = transition('linear', startState, endState, frame)
  const testCurveResult = transition(testCurve, startState, endState, frame)

  it('Transition result must be an array (linearResult)', () => {
    expect(linearResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result must be an array (testCurveResult)', () => {
    expect(testCurveResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result first element must equal startstate (linearResult)', () => {
    expect(linearResult).to.deep.property('0', startState)
  })

  it('Transition result first element must equal startstate (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property('0', startState)
  })

  it('Transition result last element must equal endState (linearResult)', () => {
    expect(linearResult).to.deep.property(frame - 1, endState)
  })

  it('Transition result last element must equal endState (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property(frame - 1, endState)
  })
})

describe('transition-Deep', () => {
  const startState = { position: [0, 0], r: 0, name: 'circle'}
  const endState = { position: [100, 100], r: 0, name: 'circle'}
  const frame = 30

  const linearResult = transition('linear', startState, endState, frame, true)
  const testCurveResult = transition(testCurve, startState, endState, frame, true)

  it('Transition result must be an array (linearResult)', () => {
    expect(linearResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result must be an array (testCurveResult)', () => {
    expect(testCurveResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result first element must equal startstate (linearResult)', () => {
    expect(linearResult).to.deep.property('0', startState)
  })

  it('Transition result first element must equal startstate (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property('0', startState)
  })

  it('Transition result last element must equal endState (linearResult)', () => {
    expect(linearResult).to.deep.property(frame - 1, endState)
  })

  it('Transition result last element must equal endState (testCurveResult)', () => {
    expect(testCurveResult).to.deep.property(frame - 1, endState)
  })
})

describe('injectNewCurve', () => {
  const startState = 0
  const endState = 100
  const frame = 30

  injectNewCurve('test', testCurve)

  const testCurveResult = transition(testCurve, startState, endState, frame)
  const injectCurveResult = transition('test', startState, endState, frame)

  it('Transition result must be an array (injectCurveResult)', () => {
    expect(injectCurveResult).to.be.an.instanceof(Array).lengthOf(frame)
  })

  it('Transition result first element must equal startstate (injectCurveResult)', () => {
    expect(injectCurveResult).to.property('0', startState)
  })

  it('Transition result last element must equal endState (injectCurveResult)', () => {
    expect(injectCurveResult).to.property(frame - 1, endState)
  })

  it('Transition result elements should all be number (injectCurveResult)', () => {
    expect(injectCurveResult).satisfy(states => {
      return !findNaNInArray(states)
    })
  })

  it('testCurveResult deep equal injectCurveResult', () => {
    expect(testCurveResult).deep.ordered.equal(injectCurveResult)
  })
})
