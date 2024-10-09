import { createWebHistory, createRouter } from 'vue-router'

import Login from './components/Login.vue'
import Overview from './components/Overview.vue'

import { isAuthenticated } from './common'

const routes = [
    {
        path: '/',
        component: Overview,
        meta: {
            requiresAuth: true
        }
    },
    {
        path: '/login',
        component: Login,
        meta: {
            requiresAuth: false
        }
    }
]

const router = createRouter({
    history: createWebHistory(),
    routes,
})

router.beforeEach((to, from) => {
    if (to.meta.requiresAuth && !isAuthenticated()) {
        return {
            path: '/login',
            // save the location the user was trying to reach so that the login page can
            // redirect to it after authentication.
            query: { redirect: to.fullPath },
        }
    }
})

export default router