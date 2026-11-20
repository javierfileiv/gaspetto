#include "LinkMonitor.h"

#include <gtest/gtest.h>

namespace
{
/* The real loop calls service() every few milliseconds, so a test that jumps
 * seconds ahead would look like a blocking animation by design. Step the clock
 * to reproduce the loop */
void serviceInSteps(LinkMonitor &link, uint32_t fromMs, uint32_t toMs)
{
    for (uint32_t now = fromMs; now <= toMs; now += 100U) {
        link.service(now);
    }
}
} /* namespace */

TEST(LinkMonitorTest, StartsUnknownAndPingsOncePerSecond)
{
    LinkMonitor link;

    EXPECT_EQ(link.state(), LinkMonitor::State::Unknown);
    /* One ping per period, the first after a second */
    EXPECT_FALSE(link.service(0));
    EXPECT_FALSE(link.service(500));
    EXPECT_TRUE(link.service(1000));
    EXPECT_FALSE(link.service(1500));
    EXPECT_TRUE(link.service(2000));
}

TEST(LinkMonitorTest, AnAnswerMakesTheLinkAlive)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(12);

    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);
    EXPECT_EQ(link.lastAnswerMs(), 12U);
}

TEST(LinkMonitorTest, OneMissedAnswerIsNotEnoughToCallItLost)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(10);
    serviceInSteps(link, 100, 2400);
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    /* 2500 ms after the last answer: exactly the window, so it is lost */
    serviceInSteps(link, 2500, 2600);
    EXPECT_EQ(link.state(), LinkMonitor::State::Lost);
}

TEST(LinkMonitorTest, AnAnswerBringsALostLinkBack)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(10);
    serviceInSteps(link, 100, 2600);
    ASSERT_EQ(link.state(), LinkMonitor::State::Lost);

    link.noteAnswer(2700);
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    /* The answer resets the window, so the link stays alive for another one */
    serviceInSteps(link, 2800, 5000);
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);
}

TEST(LinkMonitorTest, LostLinkKeepsPinging)
{
    LinkMonitor link;
    bool pinged = false;

    link.service(0);
    link.noteAnswer(10);
    serviceInSteps(link, 100, 2600);
    ASSERT_EQ(link.state(), LinkMonitor::State::Lost);

    for (uint32_t now = 2700; now <= 3600; now += 100U) {
        if (link.service(now)) {
            pinged = true;
        }
    }

    EXPECT_TRUE(pinged);
    EXPECT_EQ(link.state(), LinkMonitor::State::Lost);
}

TEST(LinkMonitorTest, ABlockingAnimationIsNotALinkLoss)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(10);

    /* Five seconds go by inside a blocking animation: the window restarts and
     * the ping that is due goes out instead of the link being called lost */
    EXPECT_TRUE(link.service(5100));
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    serviceInSteps(link, 5200, 7500);
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    serviceInSteps(link, 7600, 7600);
    EXPECT_EQ(link.state(), LinkMonitor::State::Lost);
}
