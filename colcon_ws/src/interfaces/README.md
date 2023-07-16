# Custom interfaces

## Custom Messages

- Create message folder
```bash
# In ws/src/package/
mkdir msg
```

- Custom message
```msg
geometry_msgs/Point center
float64 radius
```

- Specified in ```CMakeList.txt```
```txt

```


## Custom service

- Create service folder
```bash
# In ws/src/package
mkdir srv
```

- Custom service
```srv
# Request
float64 x
---
bool equal
```