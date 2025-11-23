# Vue3 FastAPI 后端服务

这是一个使用 FastAPI 构建的后端服务，专为 Vue3 前端应用设计。

## 功能特性

- ✅ RESTful API 设计
- ✅ CORS 跨域支持
- ✅ 用户管理 CRUD 操作
- ✅ 自动 API 文档生成
- ✅ 类型安全的数据验证

## 安装依赖

```bash
cd backend
pip install -r requirements.txt
```

## 运行服务

### 方式一：使用 uvicorn 直接运行
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 方式二：使用 Python 运行
```bash
cd backend
python main.py
```

## API 文档

服务启动后，可以访问以下地址查看自动生成的 API 文档：

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## API 端点

### 基础端点
- `GET /` - 欢迎信息
- `GET /api/health` - 健康检查

### 用户管理
- `GET /api/users` - 获取所有用户
- `GET /api/users/{user_id}` - 根据ID获取用户
- `POST /api/users` - 创建新用户
- `PUT /api/users/{user_id}` - 更新用户信息
- `DELETE /api/users/{user_id}` - 删除用户

## 数据模型

### User 模型
```json
{
  "id": 1,
  "name": "张三",
  "email": "zhangsan@example.com",
  "age": 25
}
```

### CreateUserRequest 模型
```json
{
  "name": "张三",
  "email": "zhangsan@example.com",
  "age": 25
}
```

## 开发说明

- 服务默认运行在 `http://localhost:8000`
- 支持 CORS，允许来自 `http://localhost:5173` 和 `http://127.0.0.1:5173` 的前端请求
- 使用内存存储模拟数据库，重启服务后数据会重置

## 与前端配合

前端 Vue3 应用可以通过以下方式调用后端 API：

```javascript
// 获取所有用户
const response = await fetch('http://localhost:8000/api/users')
const users = await response.json()

// 创建新用户
const newUser = await fetch('http://localhost:8000/api/users', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    name: '新用户',
    email: 'new@example.com',
    age: 30
  })
})
