# 绘图效果

流程图：
```flow
st=>start: Start
op=>operation: Your Operation
cond=>condition: Yes or No?
e=>end

st->op->cond
cond(yes)->e
cond(no)->op
```

时序图1：
```mermaid
sequenceDiagram
PythonAPI ->> LibCarla: Hello Bob, how are you?
LibCarla-->>CarlaUnreal: How about you CarlaUnreal?
LibCarla--x PythonAPI: I am good thanks!
LibCarla-x CarlaUnreal: I am good thanks!
Note right of CarlaUnreal: Bob thinks a long<br/>long time, so long<br/>that the text does<br/>not fit on a row.
 
LibCarla-->PythonAPI: Checking with CarlaUnreal...
PythonAPI->CarlaUnreal: Yes... CarlaUnreal, how are you?
```

时序图2：
```mermaid
sequenceDiagram
loop Daily query
    Alice->>Bob: Hello Bob, how are you?
    alt is sick
        Bob->>Alice: Not so good :(
    else is well
        Bob->>Alice: Feeling fresh like a daisy
    end
 
    opt Extra response
        Bob->>Alice: Thanks for asking
    end
end
```

参考[链接1](https://blog.csdn.net/chuhanning9279/article/details/100970178) 、[链接2](https://16314-mkdocs-demo.readthedocs.io/en/latest/#4) 。
