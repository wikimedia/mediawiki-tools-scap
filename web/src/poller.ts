import { Ref } from 'vue';

/**
 * Poller performs API polling for multiple subscribers and propagates
 * response data via a given Vue ref.
 */
export class Poller {
	private id: number = 0;
	private subscribers: number = 0;

	constructor(
		private ref: Ref<any>,
		private readonly update: () => Promise<any>,
		private readonly interval: number = 1000,
	) {}

	/**
	 * Increments the number of subscribers and starts polling.
	 */
	public async start() {
		if (this.id === 0) {
			this.id = setInterval(async () => await this.doUpdate(), this.interval);
			this.subscribers = 0;
			this.doUpdate();
		}

		this.subscribers++;
	}

	/**
	 * Decrements the number of subscribers and stops polling when there are no
	 * subscribers left.
	 */
	public async stop() {
		this.subscribers--;

		if (this.subscribers <= 0) {
			clearInterval(this.id);
			this.id = 0;
		}
	}

	private async doUpdate() {
		this.ref.value = await this.update();
	}
}
