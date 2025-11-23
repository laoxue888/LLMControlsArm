<script setup lang="ts">
import { ref, onMounted } from 'vue'

// 用户接口定义
interface User {
  id: number
  name: string
  email: string
  age?: number
}

// 响应式数据
const users = ref<User[]>([])
const loading = ref(false)
const error = ref('')
const newUser = ref({
  name: '',
  email: '',
  age: ''
})
const editingUser = ref<User | null>(null)

// API基础URL
const API_BASE = 'http://localhost:8000/api'

// 获取所有用户
const fetchUsers = async () => {
  loading.value = true
  error.value = ''
  try {
    const response = await fetch(`${API_BASE}/users`)
    if (!response.ok) {
      throw new Error('获取用户列表失败')
    }
    users.value = await response.json()
  } catch (err) {
    error.value = err instanceof Error ? err.message : '未知错误'
  } finally {
    loading.value = false
  }
}

// 创建新用户
const createUser = async () => {
  if (!newUser.value.name || !newUser.value.email) {
    error.value = '姓名和邮箱是必填项'
    return
  }

  loading.value = true
  error.value = ''
  try {
    const response = await fetch(`${API_BASE}/users`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        name: newUser.value.name,
        email: newUser.value.email,
        age: newUser.value.age ? parseInt(newUser.value.age) : undefined
      })
    })

    if (!response.ok) {
      throw new Error('创建用户失败')
    }

    const createdUser = await response.json()
    users.value.push(createdUser)
    
    // 重置表单
    newUser.value = { name: '', email: '', age: '' }
    error.value = ''
  } catch (err) {
    error.value = err instanceof Error ? err.message : '未知错误'
  } finally {
    loading.value = false
  }
}

// 更新用户
const updateUser = async () => {
  if (!editingUser.value) return

  loading.value = true
  error.value = ''
  try {
    const response = await fetch(`${API_BASE}/users/${editingUser.value.id}`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        name: editingUser.value.name,
        email: editingUser.value.email,
        age: editingUser.value.age
      })
    })

    if (!response.ok) {
      throw new Error('更新用户失败')
    }

    const updatedUser = await response.json()
    const index = users.value.findIndex(u => u.id === updatedUser.id)
    if (index !== -1) {
      users.value[index] = updatedUser
    }
    
    editingUser.value = null
    error.value = ''
  } catch (err) {
    error.value = err instanceof Error ? err.message : '未知错误'
  } finally {
    loading.value = false
  }
}

// 删除用户
const deleteUser = async (userId: number) => {
  if (!confirm('确定要删除这个用户吗？')) return

  loading.value = true
  error.value = ''
  try {
    const response = await fetch(`${API_BASE}/users/${userId}`, {
      method: 'DELETE'
    })

    if (!response.ok) {
      throw new Error('删除用户失败')
    }

    users.value = users.value.filter(u => u.id !== userId)
    error.value = ''
  } catch (err) {
    error.value = err instanceof Error ? err.message : '未知错误'
  } finally {
    loading.value = false
  }
}

// 开始编辑用户
const startEdit = (user: User) => {
  editingUser.value = { ...user }
}

// 取消编辑
const cancelEdit = () => {
  editingUser.value = null
}

// 组件挂载时获取用户列表
onMounted(() => {
  fetchUsers()
})
</script>

<template>
  <main class="user-management">
    <h1>用户管理系统</h1>
    
    <!-- 错误提示 -->
    <div v-if="error" class="error-message">
      {{ error }}
    </div>

    <!-- 添加用户表单 -->
    <div class="user-form">
      <h2>添加新用户</h2>
      <form @submit.prevent="createUser">
        <div class="form-group">
          <label for="name">姓名:</label>
          <input 
            id="name"
            v-model="newUser.name" 
            type="text" 
            placeholder="请输入姓名" 
            required
          >
        </div>
        <div class="form-group">
          <label for="email">邮箱:</label>
          <input 
            id="email"
            v-model="newUser.email" 
            type="email" 
            placeholder="请输入邮箱" 
            required
          >
        </div>
        <div class="form-group">
          <label for="age">年龄:</label>
          <input 
            id="age"
            v-model="newUser.age" 
            type="number" 
            placeholder="请输入年龄"
          >
        </div>
        <button type="submit" :disabled="loading">
          {{ loading ? '创建中...' : '创建用户' }}
        </button>
      </form>
    </div>

    <!-- 用户列表 -->
    <div class="user-list">
      <h2>用户列表</h2>
      <div v-if="loading && users.length === 0" class="loading">
        加载中...
      </div>
      <div v-else-if="users.length === 0" class="no-users">
        暂无用户数据
      </div>
      <div v-else class="users">
        <div v-for="user in users" :key="user.id" class="user-card">
          <div v-if="editingUser?.id === user.id" class="edit-form">
            <input v-model="editingUser.name" type="text">
            <input v-model="editingUser.email" type="email">
            <input v-model="editingUser.age" type="number">
            <div class="edit-actions">
              <button @click="updateUser" :disabled="loading">保存</button>
              <button @click="cancelEdit" :disabled="loading">取消</button>
            </div>
          </div>
          <div v-else class="user-info">
            <h3>{{ user.name }}</h3>
            <p>邮箱: {{ user.email }}</p>
            <p v-if="user.age">年龄: {{ user.age }}</p>
            <p>ID: {{ user.id }}</p>
            <div class="user-actions">
              <button @click="startEdit(user)" :disabled="loading">编辑</button>
              <button @click="deleteUser(user.id)" :disabled="loading" class="delete-btn">
                删除
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </main>
</template>

<style scoped>
.user-management {
  max-width: 800px;
  margin: 0 auto;
  padding: 20px;
}

h1 {
  text-align: center;
  color: #2c3e50;
  margin-bottom: 30px;
}

.error-message {
  background-color: #fee;
  color: #c33;
  padding: 10px;
  border-radius: 4px;
  margin-bottom: 20px;
  border: 1px solid #fcc;
}

.user-form {
  background: #f8f9fa;
  padding: 20px;
  border-radius: 8px;
  margin-bottom: 30px;
}

.form-group {
  margin-bottom: 15px;
}

label {
  display: block;
  margin-bottom: 5px;
  font-weight: bold;
  color: #2c3e50;
}

input {
  width: 100%;
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
}

button {
  padding: 8px 16px;
  background-color: #3498db;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  margin-right: 10px;
}

button:disabled {
  background-color: #bdc3c7;
  cursor: not-allowed;
}

button:hover:not(:disabled) {
  background-color: #2980b9;
}

.delete-btn {
  background-color: #e74c3c;
}

.delete-btn:hover:not(:disabled) {
  background-color: #c0392b;
}

.user-list {
  background: white;
  border-radius: 8px;
  padding: 20px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.loading, .no-users {
  text-align: center;
  padding: 40px;
  color: #7f8c8d;
}

.user-card {
  border: 1px solid #ecf0f1;
  border-radius: 6px;
  padding: 15px;
  margin-bottom: 15px;
  background: #f8f9fa;
}

.user-info h3 {
  margin: 0 0 10px 0;
  color: #2c3e50;
}

.user-info p {
  margin: 5px 0;
  color: #7f8c8d;
}

.user-actions {
  margin-top: 10px;
}

.edit-form {
  display: grid;
  gap: 10px;
}

.edit-actions {
  display: flex;
  gap: 10px;
  margin-top: 10px;
}
</style>
