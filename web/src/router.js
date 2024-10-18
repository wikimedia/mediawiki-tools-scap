import { createWebHistory, createRouter } from 'vue-router'

import Login from './components/Login.vue'
import Overview from './components/Overview.vue'
import JobViewer from './components/JobViewer.vue'

import useApi from './api';

const routes = [
    {
        path: '/',
        component: Overview,
        meta: {
            requiresAuth: true,
            title: "Welcome to SpiderPig"
        }
    },
    {
        path: '/login',
        component: Login,
        meta: {
            requiresAuth: false,
            title: "Welcome to SpiderPig"
        }
    },
    {
        path: '/jobs/:job_id',
        component: JobViewer,
        props: true,
        meta: {
            requiresAuth: true,
            title: "Welcome to job #",
        }
    },

]

const router = createRouter({
    history: createWebHistory(),
    routes,
})

router.beforeEach((to, from) => {
    const api = useApi()

    if (to.meta.requiresAuth && !api.isAuthenticated) {
        return {
            path: '/login',
            // save the location the user was trying to reach so that the login page can
            // redirect to it after authentication.
            query: { redirect: to.fullPath },
        }
    }
})

export default router