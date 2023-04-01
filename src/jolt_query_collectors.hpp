#pragma once

template<typename TBase, int32_t TInlineCapacity>
class JoltQueryCollectorAll : public TBase {
public:
	using Hit = typename TBase::ResultType;

	bool had_hit() const { return !hits.is_empty(); }

	int32_t get_hit_count() const { return hits.size(); }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

	void reset() { Reset(); }

private:
	void Reset() override {
		TBase::Reset();

		hits.clear();
	}

	void AddHit(const Hit& p_hit) override { hits.push_back(p_hit); }

	InlineVector<Hit, TInlineCapacity> hits;
};

template<typename TBase>
class JoltQueryCollectorAny final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	bool had_hit() const { return valid; }

	const Hit& get_hit() const { return hit; }

	void reset() { Reset(); }

private:
	void Reset() override {
		TBase::Reset();

		valid = false;
	}

	void AddHit(const Hit& p_hit) override {
		hit = p_hit;
		valid = true;

		TBase::ForceEarlyOut();
	}

	Hit hit;

	bool valid = false;
};

template<typename TBase, int32_t TInlineCapacity>
class JoltQueryCollectorAnyMulti final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	explicit JoltQueryCollectorAnyMulti(int32_t p_max_hits = TInlineCapacity)
		: max_hits(p_max_hits) { }

	bool had_hit() const { return hits.size() > 0; }

	int32_t get_hit_count() const { return hits.size(); }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

	void reset() { Reset(); }

private:
	void Reset() override {
		TBase::Reset();

		hits.clear();
	}

	void AddHit(const Hit& p_hit) override {
		if (hits.size() < max_hits) {
			hits.push_back(p_hit);
		}

		if (hits.size() == max_hits) {
			TBase::ForceEarlyOut();
		}
	}

	InlineVector<Hit, TInlineCapacity> hits;

	int32_t max_hits = 0;
};

template<typename TBase>
class JoltQueryCollectorClosest final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	bool had_hit() const { return valid; }

	const Hit& get_hit() const { return hit; }

	void reset() { Reset(); }

private:
	void Reset() override {
		TBase::Reset();

		valid = false;
	}

	void AddHit(const Hit& p_hit) override {
		const float early_out = p_hit.GetEarlyOutFraction();

		if (!valid || early_out < hit.GetEarlyOutFraction()) {
			TBase::UpdateEarlyOutFraction(early_out);

			hit = p_hit;
			valid = true;
		}
	}

	Hit hit;

	bool valid = false;
};

template<typename TBase, int32_t TInlineCapacity>
class JoltQueryCollectorClosestMulti final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	explicit JoltQueryCollectorClosestMulti(int32_t p_max_hits = TInlineCapacity)
		: max_hits(p_max_hits) { }

	bool had_hit() const { return hits.size() > 0; }

	int32_t get_hit_count() const { return hits.size(); }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

	void reset() { Reset(); }

private:
	void Reset() override {
		TBase::Reset();

		hits.clear();
	}

	void AddHit(const Hit& p_hit) override {
		hits.ordered_insert(p_hit, [](const Hit& p_lhs, const Hit& p_rhs) {
			return p_lhs.GetEarlyOutFraction() < p_rhs.GetEarlyOutFraction();
		});

		if (hits.size() > max_hits) {
			hits.resize(max_hits);
		}
	}

	InlineVector<Hit, TInlineCapacity + 1> hits;

	int32_t max_hits = 0;
};
