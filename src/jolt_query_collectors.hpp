#pragma once

template<typename TBase>
class JoltQueryCollectorAll : public TBase {
public:
	using Hit = typename TBase::ResultType;

	bool had_hit() const { return !hits.is_empty(); }

	int32_t get_hit_count() const { return hits.size(); }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

private:
	void Reset() override {
		TBase::Reset();

		hits.clear();
	}

	void AddHit(const Hit& p_hit) override { hits.push_back(p_hit); }

	InlineVector<Hit, 32> hits;
};

template<typename TBase>
class JoltQueryCollectorAny final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	explicit JoltQueryCollectorAny(int32_t p_max_hits)
		: hits(p_max_hits) {
		hits.resize(p_max_hits);
	}

	bool had_hit() const { return hit_count > 0; }

	int32_t get_hit_count() const { return hit_count; }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

private:
	void Reset() override {
		TBase::Reset();

		hit_count = 0;
	}

	void AddHit(const Hit& p_hit) override {
		if (hit_count < hits.size()) {
			hits[hit_count++] = p_hit;
		}

		if (hit_count == hits.size()) {
			TBase::ForceEarlyOut();
		}
	}

	InlineVector<Hit, 32> hits;

	int32_t hit_count = 0;
};

template<typename TBase>
class JoltQueryCollectorClosest final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	bool had_hit() const { return valid; }

	const Hit& get_hit() const { return hit; }

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

template<typename TBase>
class JoltQueryCollectorClosestMulti final : public TBase {
public:
	using Hit = typename TBase::ResultType;

	explicit JoltQueryCollectorClosestMulti(int32_t p_max_hits)
		: max_hits(p_max_hits) { }

	bool had_hit() const { return hits.size() > 0; }

	int32_t get_hit_count() const { return hits.size(); }

	const Hit& get_hit(int32_t p_index) const { return hits[p_index]; }

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

	InlineVector<Hit, 33> hits;

	int32_t max_hits = 0;
};
