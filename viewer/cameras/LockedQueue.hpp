#pragma  once
#include <mutex>
#include <condition_variable>
#include <deque>

// Consumer and Producer
template <class T>
struct LockedQueue {
	// コンストラクタに、キューの大きさを指定
	explicit LockedQueue(int capacity)
		: capacity(capacity)
	{}

	// キューにxを加える
	void enqueue(const T& x) {
		std::unique_lock<std::mutex> lock(m);
		// 外からnotify_all()またはnotify_one()によって起こされるまでブロックして待つ
		// ただし、起こされた時にキューが満杯だった場合は、またブロックして待つ
		c_enq.wait(lock, [this] { return data.size() != capacity; });
		data.push_back(x);
		// dequeueの準備ができたことを通知
		c_deq.notify_one();
	}

	// キューから要素を取り出す
	T dequeue() {
		std::unique_lock<std::mutex> lock(m);
		// 外からnotify_all()またはnotify_one()によって起こされるまでブロックして待つ
		// ただし、起こされた時にキューが空である場合は、またブロックして待つ
		c_deq.wait(lock, [this] { return !data.empty(); });
		T ret = data.front();
		data.pop_front();
		// enqueueの準備ができたことを通知
		c_enq.notify_one();
		return ret;
	}

	// data size の取得
	size_t getCurrentDataSize()
	{
		return data.size();
	}

private:
	std::mutex m;
	std::deque<T> data;
	size_t capacity;
	std::condition_variable c_enq;
	std::condition_variable c_deq;
};