import { createWebHistory, createRouter } from 'vue-router';
import AdminPage from './components/AdminPage.vue';
import LoginPage from './components/LoginPage.vue';
import OverviewPage from './components/OverviewPage.vue';
import JobViewerPage from './components/JobViewerPage.vue';
import NotAuthorized from './components/NotAuthorized.vue';

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
	},
	{
		name: 'admin',
		path: '/admin',
		component: AdminPage
	},
	{
		name: 'notauthorized',
		path: '/notauthorized',
		component: NotAuthorized
	}
];

const router = createRouter( {
	history: createWebHistory(),
	routes
} );

export default router;
