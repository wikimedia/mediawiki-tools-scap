import { createWebHistory, createRouter } from 'vue-router';
import LoginPage from './components/LoginPage.vue';
import OverviewPage from './components/OverviewPage.vue';
import JobViewerPage from './components/JobViewerPage.vue';
import useApi from './api';

const routes = [
	{
		path: '/',
		component: OverviewPage,
		meta: {
			requiresAuth: true,
			title: 'Welcome to SpiderPig'
		}
	},
	{
		path: '/login',
		component: LoginPage,
		meta: {
			requiresAuth: false,
			title: 'Welcome to SpiderPig'
		}
	},
	{
		name: 'job',
		path: '/jobs/:jobId',
		component: JobViewerPage,
		props: true,
		meta: {
			requiresAuth: true,
			title: 'Welcome to job #'
		}
	}
];

const router = createRouter( {
	history: createWebHistory(),
	routes
} );

router.beforeEach( ( to, _from ) => {
	const api = useApi();

	if ( to.meta.requiresAuth && !api.isAuthenticated ) {
		return {
			path: '/login',
			// save the location the user was trying to reach so that the login page can
			// redirect to it after authentication.
			query: { redirect: to.fullPath }
		};
	}
} );

export default router;
