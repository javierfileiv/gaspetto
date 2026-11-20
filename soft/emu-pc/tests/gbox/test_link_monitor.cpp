#include "LinkMonitor.h"

#include <gtest/gtest.h>

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

    EXPECT_TRUE(link.service(2000));
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    /* 2500 ms after the last answer: exactly the window, so it is lost */
    EXPECT_FALSE(link.service(2510));
    EXPECT_EQ(link.state(), LinkMonitor::State::Lost);
}

TEST(LinkMonitorTest, AnAnswerBringsALostLinkBack)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(10);
    link.service(2510);
    ASSERT_EQ(link.state(), LinkMonitor::State::Lost);

    link.noteAnswer(2600);
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);

    /* The answer resets the window, so the link stays alive for another one */
    EXPECT_FALSE(link.service(3000));
    EXPECT_EQ(link.state(), LinkMonitor::State::Alive);
}

TEST(LinkMonitorTest, LostLinkKeepsPinging)
{
    LinkMonitor link;

    link.service(0);
    link.noteAnswer(10);
    /* The window expires and the ping period is over at the same call */
    EXPECT_TRUE(link.service(2510));
    ASSERT_EQ(link.state(), LinkMonitor::State::Lost);

    EXPECT_FALSE(link.service(3010));
    EXPECT_TRUE(link.service(3510));
    EXPECT_EQ(link.state(), LinkMonitor::State::Lost);
}
