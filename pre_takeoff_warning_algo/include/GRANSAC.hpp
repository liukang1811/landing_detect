#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <memory>
#include <algorithm>
#include <vector>
#include <omp.h>

#include "AbstractModel.hpp"

namespace GRANSAC
{
	// T - AbstractModel
	template <class T, int t_NumParams>
	class RANSAC
	{
	private:
		std::vector<std::shared_ptr<AbstractParameter>> m_Data; // All the data

		std::vector<std::shared_ptr<T>> m_SampledModels; // 每次循环得出一个模型参数 参数存储在m_SampledModels
		std::shared_ptr<T> m_BestModel; // Pointer to the best model, valid only after Estimate() is called
		std::vector<std::shared_ptr<AbstractParameter>> m_BestInliers;//最好循环的模型中的点

		int m_MaxIterations; // Number of iterations before termination最大循环次数
		VPFloat m_Threshold; // The threshold for computing model consensus判断点是否在模型的阈值

		int m_BestModelIdx;//存储最好模型的号

		std::vector<std::mt19937> m_RandEngines; // Mersenne twister high quality RNG that support *OpenMP* multi-threading

	public:
		RANSAC(void)
		{
            int nThreads = std::max(1, omp_get_max_threads());
            //std::cout << "[ INFO ]: Maximum usable threads: " << nThreads << std::endl;
            for (int i = 0; i < nThreads; ++i)
            {
                std::random_device SeedDevice;
                m_RandEngines.push_back(std::mt19937(SeedDevice()));
            }

			Reset();
		};

		virtual ~RANSAC(void) {};

		void Reset(void)
		{
			// Clear sampled models, etc. and prepare for next call. Reset RANSAC estimator state
			m_Data.clear();
			m_SampledModels.clear();

			m_BestModelIdx = -1;
			m_BestModelScore = 0.0;
		};

		void Initialize(VPFloat Threshold, int MaxIterations = 1000)
		{
			m_Threshold = Threshold;
			m_MaxIterations = MaxIterations;
		};
        VPFloat m_BestModelScore; // The score of the best model 局内点/全部点
		std::shared_ptr<T> GetBestModel(void) { return m_BestModel; };//获取最优模型参数
		const std::vector<std::shared_ptr<AbstractParameter>>& GetBestInliers(void) { return m_BestInliers; };//获取最优模型参数对应的点

		bool Estimate(const std::vector<std::shared_ptr<AbstractParameter>> &Data)
		{
			if (Data.size() <= t_NumParams)
			{
				std::cerr << "[ WARN ]: RANSAC - Number of data points is too less. Not doing anything." << std::endl;
				return false;
			}

			m_Data = Data;
			int DataSize = m_Data.size();
			std::uniform_int_distribution<int> UniDist(0, int(DataSize - 1)); // Both inclusive

			std::vector<VPFloat> InlierFractionAccum(m_MaxIterations);//初始化用于存储评估分数的数组
			std::vector<std::vector<std::shared_ptr<AbstractParameter>>> InliersAccum(m_MaxIterations);//初始化用于存储局内点的数组
			m_SampledModels.resize(m_MaxIterations);

            int nThreads = std::max(1, omp_get_max_threads());
            omp_set_dynamic(0); // Explicitly disable dynamic teams
            omp_set_num_threads(nThreads);
     #pragma omp parallel for
			for (int i = 0; i < m_MaxIterations; ++i)
			{
				// Select t_NumParams random samples
				std::vector<std::shared_ptr<AbstractParameter>> RandomSamples(t_NumParams);//用于存储将要计算的点
				std::vector<std::shared_ptr<AbstractParameter>> RemainderSamples = m_Data; // Without the chosen random samples
                //以下通过shuff函数打乱顺序，然后拷贝前两个到RandomSamples
				std::shuffle(RemainderSamples.begin(), RemainderSamples.end(), m_RandEngines[omp_get_thread_num()]); // To avoid picking the same element more than once
				std::copy(RemainderSamples.begin(), RemainderSamples.begin() + t_NumParams, RandomSamples.begin());
				//RemainderSamples.erase(RemainderSamples.begin(), RemainderSamples.begin() + t_NumParams); // Remove the model data points from consideration. 2018: Turns out this is not a good idea
				//将RandomSamples中的数据导入到RandomModel,在类T的构造函数中求解参数
				std::shared_ptr<T> RandomModel = std::make_shared<T>(RandomSamples);
                
				// Check if the sampled model is the best so far评估参数是否为最优
				std::pair<VPFloat, std::vector<std::shared_ptr<AbstractParameter>>> EvalPair = RandomModel->Evaluate(RemainderSamples, m_Threshold);
				InlierFractionAccum[i] = EvalPair.first;//得到当前参数下优劣的评估值
				InliersAccum[i] = EvalPair.second;//得到当前参数下的最优点

				// Push back into history. Could be removed later
				m_SampledModels[i] = RandomModel;//得到当前的参数并用vector存储
			}
            //遍历所有的数组根据评估参数选取最优参数估计
			for (int i = 0; i < m_MaxIterations; ++i)
			{
				if (InlierFractionAccum[i] > m_BestModelScore)
				{
					m_BestModelScore = InlierFractionAccum[i];
					m_BestModelIdx = m_SampledModels.size() - 1;
					m_BestModel = m_SampledModels[i];
					m_BestInliers = InliersAccum[i];
				}
			}

			//std::cerr << "BestInlierFraction: " << m_BestModelScore << std::endl;

			Reset();

			return true;
		};
	};
} // namespace GRANSAC
