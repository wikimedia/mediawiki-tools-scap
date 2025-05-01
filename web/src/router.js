import { createWebHistory, createRouter } from 'vue-router';
import AdminPage from './components/AdminPage.vue';
import LoginPage from './components/LoginPage.vue';
import JobViewerPage from './components/JobViewerPage.vue';
import NotAuthorized from './components/NotAuthorized.vue';
import BackportPage from './components/BackportPage.vue';
import TrainPage from './components/TrainPage.vue';

const routes = [
	{
		path: '/',
		component: BackportPage
	},
	{
		path: '/mediawiki/backport',
		component: BackportPage
	},
	{
		path: '/mediawiki/train',
		component: TrainPage
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
