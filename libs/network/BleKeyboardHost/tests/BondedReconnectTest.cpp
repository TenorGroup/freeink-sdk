#include <gtest/gtest.h>

#include <cstring>

#include "BleKeyboardHost.h"
#include "FakeBle.h"
#include "HidDescriptors.h"

using namespace freeink;

namespace {

constexpr char kA[] = "AA:BB:CC:DD:EE:01";
constexpr char kB[] = "AA:BB:CC:DD:EE:02";
constexpr char kC[] = "AA:BB:CC:DD:EE:03";
constexpr char kUnbonded[] = "AA:BB:CC:DD:EE:04";

class BondedReconnectTest : public ::testing::Test {
 protected:
  void SetUp() override {
    fakeble::resetWorld();
    PairedHidDevice bonds[3] = {};
    strcpy(bonds[0].addr, kA);
    strcpy(bonds[1].addr, kB);
    strcpy(bonds[2].addr, kC);
    fakeble::state().nvs["freeink-hid/n"] = {3};
    const auto* bytes = reinterpret_cast<const uint8_t*>(bonds);
    fakeble::state().nvs["freeink-hid/b"] = {bytes, bytes + sizeof bonds};
    const int map = fakeble::addCharacteristic(0x2A4B, true);
    fakeble::setCharacteristicValue(map, hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
    fakeble::addCharacteristic(0x2A4D, false, false, true);
    ASSERT_TRUE(fakeble::beginHost());
  }
  void TearDown() override { fakeble::endHost(); }
  void arm(PickPolicy policy, const char* priority) {
    ASSERT_TRUE(fakeble::host().armBondedReconnect(policy, priority));
    poll();
    ASSERT_TRUE(fakeble::host().isScanning());
    EXPECT_EQ(fakeble::state().connectCalls, 0u);
  }
  void ingest(const char* addr, uint8_t type = 1, bool connectable = true) {
    const size_t before = fakeble::allocationCount();
    fakeble::host().onScanResultIngest(addr, nullptr, -40, type, false, connectable);
    EXPECT_EQ(fakeble::allocationCount(), before);
    EXPECT_EQ(fakeble::host().deviceCount(), 0u);
  }
  void poll() {
    fakeble::host().poll();
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
  }
  void expectOnly(const char* addr) {
    EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{addr});
    EXPECT_FALSE(fakeble::host().isScanning());
  }
};

TEST_F(BondedReconnectTest, FirstConnectsOnlyAdvertisingBond) {
  arm(PickPolicy::First, kA);
  ingest(kB);
  poll();
  expectOnly(kB);
  EXPECT_EQ(fakeble::state().connectAddressTypes, std::vector<uint8_t>{1});
}

TEST_F(BondedReconnectTest, PriorityAbsentWaitsExactlyThreeSecondsFromFirstAlternative) {
  arm(PickPolicy::Priority, kA);
  ingest(kB);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  fakeble::advanceMillis(2999);
  ingest(kB);
  ingest(kC);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  fakeble::advanceMillis(1);
  poll();
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, AdvertisingPriorityConnectsImmediately) {
  arm(PickPolicy::Priority, kB);
  ingest(kB);
  poll();
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, PriorityArrivingFiveHundredMsAfterAlternativeWins) {
  arm(PickPolicy::Priority, kA);
  ingest(kB);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  fakeble::advanceMillis(500);
  ingest(kA);
  poll();
  expectOnly(kA);
}

TEST_F(BondedReconnectTest, FirstKeepsAlternativeWhenPriorityArrivesLater) {
  arm(PickPolicy::First, kA);
  ingest(kB);
  fakeble::advanceMillis(500);
  ingest(kA);
  poll();
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, UnbondedAdvertisersNeverConnectOrEnterPairingTable) {
  for (const auto policy : {PickPolicy::Priority, PickPolicy::First}) {
    if (policy == PickPolicy::First) {
      ASSERT_TRUE(fakeble::host().end());
      ASSERT_TRUE(fakeble::beginHost());
    }
    arm(policy, kA);
    const size_t before = fakeble::allocationCount();
    fakeble::host().onScanResultIngest(kUnbonded, "Unknown HID", -30, 0, true, true);
    EXPECT_EQ(fakeble::allocationCount(), before);
    fakeble::state().advertise(kUnbonded, "Unknown HID", -30);
    fakeble::advanceMillis(4000);
    poll();
    EXPECT_EQ(fakeble::state().connectCalls, 0u);
    EXPECT_EQ(fakeble::host().deviceCount(), 0u);
    EXPECT_EQ(fakeble::state().retainedScanResults(), 0u);
  }
}

TEST_F(BondedReconnectTest, NonConnectableBondIsIgnored) {
  arm(PickPolicy::First, kA);
  ingest(kB, 1, false);
  fakeble::advanceMillis(4000);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(BondedReconnectTest, ScanCallbackSelectsBondWithoutPairingTable) {
  arm(PickPolicy::First, kA);
  fakeble::state().advertise(kB, "Bond B", -30);
  poll();
  expectOnly(kB);
  EXPECT_EQ(fakeble::host().deviceCount(), 0u);
}

TEST_F(BondedReconnectTest, MissingOrUnbondedPriorityActsAsFirst) {
  for (const char* priority : {static_cast<const char*>(nullptr), "", kUnbonded}) {
    ASSERT_TRUE(fakeble::host().armBondedReconnect(PickPolicy::Priority, priority));
    poll();
    ingest(kB);
    poll();
    ASSERT_EQ(fakeble::state().connectAddresses.back(), kB);
    ASSERT_TRUE(fakeble::host().end());
    ASSERT_TRUE(fakeble::beginHost());
  }
  EXPECT_EQ(fakeble::state().connectCalls, 3u);
}

TEST_F(BondedReconnectTest, FailedKnownTypeReturnsToScanWithoutAlternateTypeRetry) {
  fakeble::setConnectSucceeds(false);
  arm(PickPolicy::First, kA);
  ingest(kB);
  poll();
  expectOnly(kB);
  EXPECT_EQ(fakeble::state().connectAddressTypes, std::vector<uint8_t>{1});
  poll();
  EXPECT_TRUE(fakeble::host().isScanning());
  fakeble::setConnectSucceeds(true);
  ingest(kC);
  poll();
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{kB, kC}));
}

TEST_F(BondedReconnectTest, AttemptDeadlineReturnsToScan) {
  arm(PickPolicy::First, kA);
  fakeble::setBlockingStage(fakeble::BlockingStage::Connect);
  ingest(kB);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Connect));
  fakeble::advanceMillis(14999);
  fakeble::host().poll();
  EXPECT_EQ(fakeble::state().cancelConnectCalls, 0u);
  fakeble::advanceMillis(1);
  poll();
  EXPECT_GT(fakeble::state().cancelConnectCalls, 0u);
  poll();
  EXPECT_TRUE(fakeble::host().isScanning());
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
}

TEST_F(BondedReconnectTest, LinkLossReturnsToScanAndCanSelectAnotherBond) {
  const auto originalNvs = fakeble::state().nvs;
  arm(PickPolicy::First, kA);
  ingest(kB);
  poll();
  ASSERT_TRUE(fakeble::host().isConnected());
  fakeble::state().client->disconnect();
  poll();
  ASSERT_TRUE(fakeble::host().isScanning());
  ingest(kC);
  poll();
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{kB, kC}));
  EXPECT_EQ(fakeble::state().nvs, originalNvs);
}

TEST_F(BondedReconnectTest, ExplicitDisconnectCancelsPlanAndQueuedAttempt) {
  arm(PickPolicy::First, kA);
  fakeble::holdWorkerNotifications(true);
  ingest(kB);
  fakeble::host().poll();
  fakeble::host().disconnect();
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::advanceMillis(4000);
  poll();
  EXPECT_FALSE(fakeble::host().isScanning());
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(BondedReconnectTest, ExplicitPairingScanCancelsBondedSelection) {
  arm(PickPolicy::First, kA);
  fakeble::host().startScan();
  fakeble::host().onScanResultIngest(kUnbonded, "Unknown HID", -30, 0, true, true);
  poll();
  EXPECT_EQ(fakeble::host().deviceCount(), 1u);
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(BondedReconnectTest, ArmingExistingLinkPreservesItAndScansAfterLoss) {
  ASSERT_EQ(fakeble::connectTo(kB), fakeble::LinkResult::Connected);
  ASSERT_TRUE(fakeble::host().armBondedReconnect(PickPolicy::Priority, kA));
  poll();
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_FALSE(fakeble::host().isScanning());
  fakeble::state().client->disconnect();
  poll();
  EXPECT_TRUE(fakeble::host().isScanning());
}

TEST_F(BondedReconnectTest, RepeatedArmAndPollNeverQueueDuplicateAttempt) {
  arm(PickPolicy::First, kA);
  EXPECT_TRUE(fakeble::host().armBondedReconnect(PickPolicy::First, kA));
  fakeble::holdWorkerNotifications(true);
  ingest(kB);
  fakeble::host().poll();
  for (unsigned count = 0; count < 10; ++count) fakeble::host().poll();
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, PriorityWaitSurvivesMillisWrap) {
  fakeble::advanceMillis(UINT32_MAX - fakeble::clockMs() - 1000);
  arm(PickPolicy::Priority, kA);
  ingest(kB);
  fakeble::advanceMillis(2999);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  fakeble::advanceMillis(1);
  poll();
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, EndClearsPlanAndNextBeginRestoresDefaultReconnect) {
  arm(PickPolicy::First, kA);
  ASSERT_TRUE(fakeble::host().end());
  EXPECT_FALSE(fakeble::host().armBondedReconnect(PickPolicy::First, kA));
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::advanceMillis(4000);
  poll();
  expectOnly(kA);
}

TEST_F(BondedReconnectTest, ForgettingPendingCandidateCannotConnectIt) {
  arm(PickPolicy::First, kA);
  ingest(kB);
  fakeble::host().forget(kB);
  poll();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  ingest(kC);
  poll();
  expectOnly(kC);
}

TEST_F(BondedReconnectTest, ForgettingPriorityImmediatelyFallsBackToFirst) {
  arm(PickPolicy::Priority, kA);
  ingest(kB);
  fakeble::host().forget(kA);
  poll();
  expectOnly(kB);
}

TEST_F(BondedReconnectTest, NewReaderPolicyAppliesWithoutDroppingExistingLink) {
  arm(PickPolicy::Priority, kA);
  ingest(kA);
  poll();
  ASSERT_TRUE(fakeble::host().isConnected());
  ASSERT_TRUE(fakeble::host().armBondedReconnect(PickPolicy::First, kA));
  EXPECT_TRUE(fakeble::host().isConnected());
  fakeble::state().client->disconnect();
  poll();
  ingest(kB);
  poll();
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{kA, kB}));
}

TEST_F(BondedReconnectTest, RepeatedReaderArmPreservesAlternativeFirstSeenTime) {
  arm(PickPolicy::Priority, kA);
  ingest(kB);
  fakeble::advanceMillis(2999);
  ASSERT_TRUE(fakeble::host().armBondedReconnect(PickPolicy::Priority, kA));
  fakeble::advanceMillis(1);
  poll();
  expectOnly(kB);
}

}  // namespace
