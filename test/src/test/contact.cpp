#include "sensorbox/contact.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <unordered_map>

#include "sensorbox/test/test_instances.hpp"

namespace sensorbox {

using Timestamp = ContactClassifications::Timestamp;

TEST(contact_classifications, default_construction) {
    const ContactClassifications classifications;
    EXPECT_EQ(classifications.timestamp(), Timestamp{ContactClassifications::Duration::zero()});
    EXPECT_TRUE(classifications.empty());
    EXPECT_EQ(classifications.size(), 0);
    EXPECT_EQ(classifications.num_contacts(), 0);
}

TEST(contact_classifications, construction_with_timestamp) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const ContactClassifications classifications{timestamp};
    EXPECT_EQ(classifications.timestamp(), timestamp);
    EXPECT_TRUE(classifications.empty());
}

TEST(contact_classifications, construction_with_classifications) {
    const Timestamp timestamp{std::chrono::nanoseconds{42}};
    const std::unordered_map<std::string, bool> initial{{test_string(0), true}, {test_string(1), false}};
    const ContactClassifications classifications{timestamp, initial};
    EXPECT_EQ(classifications.timestamp(), timestamp);
    EXPECT_FALSE(classifications.empty());
    EXPECT_EQ(classifications.size(), 2);
    EXPECT_EQ(classifications.num_contacts(), 1);
    EXPECT_TRUE(classifications.has_classification(test_string(0)));
    EXPECT_TRUE(classifications.has_classification(test_string(1)));
    EXPECT_FALSE(classifications.has_classification(test_string(2)));
    EXPECT_TRUE(classifications.classifications().at(test_string(0)));
    EXPECT_FALSE(classifications.classifications().at(test_string(1)));
}

TEST(contact_classifications, set_classification) {
    ContactClassifications classifications;
    classifications.set_classification(test_string(0), true);
    classifications.set_classification(test_string(1), true);
    classifications.set_classification(test_string(2), false);
    EXPECT_EQ(classifications.size(), 3);
    EXPECT_EQ(classifications.num_contacts(), 2);
    // Setting an existing link overwrites its classification
    classifications.set_classification(test_string(0), false);
    EXPECT_EQ(classifications.size(), 3);
    EXPECT_EQ(classifications.num_contacts(), 1);
    EXPECT_FALSE(classifications.classifications().at(test_string(0)));
}

TEST(contact_classifications, mutable_access) {
    ContactClassifications classifications;
    classifications.classifications()[test_string(0)] = true;
    EXPECT_EQ(classifications.num_contacts(), 1);
    classifications.classifications().clear();
    EXPECT_TRUE(classifications.empty());
}

}
