import { createWebHistory, createRouter } from 'vue-router';
import LoginPage from './components/LoginPage.vue';
import OverviewPage from './components/OverviewPage.vue';
import JobViewerPage from './components/JobViewerPage.vue';

const routes = [
	{
		path: '/',
		component: OverviewPage
	},
	{
		path: '/login',
		component: LoginPage
	},
	{
		name: 'job',
		path: '/jobs/:jobId',
		component: JobViewerPage,
		props: true
	}
];

const router = createRouter( {
	history: createWebHistory(),
	routes
} );

export default router;
