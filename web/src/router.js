import { createWebHistory, createRouter } from 'vue-router'

import Overview from './components/Overview.vue'

const routes = [
  { path: '/', component: Overview },
]

const router = createRouter({
  history: createWebHistory(),
  routes,
})

export default router