#ifndef PHAQM_QUEUE_DISC_H
#define PHAQM_QUEUE_DISC_H

#include "ns3/queue-disc.h"
#include "ns3/core-module.h"
#include "ns3/queue-size.h"
#include <vector>

namespace ns3 {

class PhaqmQueueDisc : public QueueDisc
{
public:
    static TypeId GetTypeId(void);

    PhaqmQueueDisc();
    virtual ~PhaqmQueueDisc();

    void SetMaxSize(const QueueSize &maxSize);
    QueueSize GetMaxSize() const;

    virtual bool CheckConfig(void) override;
    virtual void InitializeParams(void) override;

protected:
    virtual bool DoEnqueue(Ptr<QueueDiscItem> item) override;
    virtual Ptr<QueueDiscItem> DoDequeue(void) override;
    virtual void DoDispose(void) override;

private:
    QueueSize m_maxSize;
    double m_queueRef;

    double m_C;   // Link capacity (packets per second)
    double m_R0;  // Round-trip time (seconds)
    uint32_t m_N; // Number of TCP flows

    double m_samplingTime; // Sampling time Ts (seconds)
    uint32_t m_delayD;     // System delay in samples d

    double m1, m2, n1, n2;

    std::vector<double> e_coeffs;
    std::vector<double> f_coeffs;

    std::vector<double> g_coeffs;
    std::vector<double> h_coeffs;

    std::vector<double> m_yHist;
    std::vector<double> m_uHist;

    Ptr<UniformRandomVariable> m_uniformRV;

    // Hebbian adaptive weights and learning rates
    std::vector<double> m_weights;
    std::vector<double> m_learningRates;
    std::vector<double> m_prevInput;
    double m_prevError;
    double m_prevControl;

    void ComputeModelParameters(double C, double R0, uint32_t N, double Ts);
    void SetDiophantineCoefficients();
    void ComputeControlCoefficients();
};

} // namespace ns3

#endif // PHAQM_QUEUE_DISC_H

