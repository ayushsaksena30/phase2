import asyncio

async def producer(queue):
    while True:
        data = "Some data"
        await queue.put(data)
        print("Produced:", data)
        await asyncio.sleep(1)  # Simulate work

async def consumer(queue):
    while True:
        data = await queue.get()
        print("Consumed:", data)
        queue.task_done()  # Mark the item as processed
        await asyncio.sleep(2)  # Simulate work

async def main():
    queue = asyncio.Queue()
    
    producer_task = asyncio.create_task(producer(queue))
    consumer_task = asyncio.create_task(consumer(queue))
    
    await asyncio.gather(producer_task, consumer_task)

# Run the main event loop
asyncio.run(main())